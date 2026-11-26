"""학습 샘플(samples.zip)로 크기·색상 판별을 검증하고 판별 기준(프로필)을 다시 맞추는 도구.

펌웨어(src/main.cpp)의 판별 과정을 그대로 재현한다:
  1) 배경 차분 → 강한 전경(그림자 필터 적용) / 약한 전경(그림자 필터 없음)
  2) 3x3 다수결 → 가장 큰 덩어리 → 색상 판정
  3) 검정 물체면 약한 전경으로 덩어리를 키움 (반사로 회색이 된 윗면·옆면 복원)
  4) 3x3 열림 → 가장 큰 덩어리 → 가장자리의 볼록 껍질
  5) 원근 보정 좌표에서 옆면(색 무리별 두께)을 뺀 최소 외접 사각형 → 길이·폭(mm)

판별 기준(프로필)은 펌웨어의 PROFILE_BUILTIN 과 같다. --fit 은 펌웨어 웹의 '학습 데이터로 기준 계산'과
같은 방법(Nelder-Mead, 같은 비용 함수·초기값)으로 새 기준을 계산한다.

사용법:
  python tools/size_check.py link-cam1_samples_2.zip               # 기본 프로필(2)로 판별 결과 보기
  python tools/size_check.py link-cam1_samples.zip --profile 1     # 프로필 1(옛 환경)로
  python tools/size_check.py link-cam1_samples_2.zip --fit         # 샘플로 새 기준 계산 (펌웨어에 넣을 값 출력)
필요 패키지: numpy, pillow
"""
import argparse
import io
import json
import math
import sys
import zipfile
from collections import deque

import numpy as np
from PIL import Image

# ---- src/main.cpp 와 같은 값 ----
DIFF_THRESH, CHROMA_THRESH, SHADOW_MIN_PCT = 60, 30, 50
PX_CHROMA_MIN, PX_SAT_MIN, CHROMA_SHARE_MIN = 25, 0.28, 0.35
HUE = (335.0, 28.0, 68.0, 165.0, 260.0)      # 빨강 시작, 노랑, 초록, 파랑, 파랑 끝
BLACK_GROW_COLS = 8
CALIPER_STEP_DEG = 3
IMG_CX, IMG_CY = 80.0, 60.0                   # 원근 보정 기준점 (전체 160x120 영상의 중심)
NOMINAL_MM = {1: 35.0, 2: 55.0, 3: 75.0}      # 소·중·대 길이 (단면은 모두 35mm)
WIDTH_MM = 35.0

# 판별 기준 (펌웨어 PROFILE_BUILTIN 과 같은 순서·값)
#   P·K·px35: 원근 보정, sideW/K/C: 흰·검정·유채색의 옆면 두께(보정 좌표), below: 검정 보강 때 덩어리 바닥 아래로 허용하는 px,
#   smallMax/mediumMax: 크기 경계(mm), wbSplit: 흰/검 밝기비 경계
PROFILES = {
    1: dict(name='학습 1 (samples.zip)', P=0.00203, K=1.1432, px35=31.48, sideW=0.0, sideK=10.0, sideC=10.0,
            below=0, smallMax=47.0, mediumMax=58.5, wbSplit=1.2),
    2: dict(name='학습 2 (samples_2.zip)', P=0.00153, K=1.2511, px35=32.19, sideW=13.21, sideK=19.34, sideC=3.20,
            below=10, smallMax=46.3, mediumMax=64.2, wbSplit=1.07),
}
DEFAULT_PROFILE = 2

SIZE_NAMES = ['미판별', '소', '중', '대']
COLOR_NAMES = ['미판별', '흰', '검', '빨', '노', '초', '파']


def load_zip(path):
    z = zipfile.ZipFile(path)
    meta = json.loads(z.read('samples.json').decode('utf-8'))
    img = lambda name: np.asarray(Image.open(io.BytesIO(z.read(name))).convert('RGB'), dtype=np.int32)
    bgs = {b['bg']: img(b['file']) for b in meta['backgrounds']}
    x0, y0 = meta['roi'][0], meta['roi'][1]
    return meta, [dict(s=s, raw=img(s['raw']), bg=bgs.get(s['bg']), x0=x0, y0=y0) for s in meta['samples'] if s['bg'] in bgs]


def majority(b):
    H, W = b.shape
    p = np.pad(b.astype(np.int32), 1)
    return sum(p[1 + dy:1 + dy + H, 1 + dx:1 + dx + W] for dy in (-1, 0, 1) for dx in (-1, 0, 1)) >= 5


def flood(mask, seeds, allow=None):
    """seeds에서 8-연결로 mask(그리고 allow)를 따라 채운 영역"""
    H, W = mask.shape
    out = np.zeros_like(mask, bool)
    q = deque()
    for y, x in seeds:
        out[y, x] = True
        q.append((y, x))
    while q:
        y, x = q.popleft()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                a, b = y + dy, x + dx
                if 0 <= a < H and 0 <= b < W and not out[a, b] and mask[a, b] and (allow is None or allow[a, b]):
                    out[a, b] = True
                    q.append((a, b))
    return out


def largest(mask):
    seen = np.zeros_like(mask, bool)
    best = np.zeros_like(mask, bool)
    for y, x in zip(*np.nonzero(mask)):
        if seen[y, x]:
            continue
        c = flood(mask, [(y, x)])
        seen |= c
        if c.sum() > best.sum():
            best = c
    return best


def foreground(raw, bg, gain):
    g8 = [int(round(g * 256)) for g in gain]
    wb = lambda a: np.stack([(a[..., k] * g8[k]) >> 8 for k in range(3)], -1)
    chroma = lambda a: wb(a).max(-1) - wb(a).min(-1)
    dsum = np.abs(raw - bg).sum(-1)
    curV, bgV = raw.max(-1), bg.max(-1)
    curC, bgC = chroma(raw), chroma(bg)
    chromaFg = (curC - bgC) >= CHROMA_THRESH
    shadow = (curV < bgV) & (curV * 100 >= bgV * SHADOW_MIN_PCT) & (np.abs(curC - bgC) < CHROMA_THRESH)
    strong = chromaFg | ((dsum >= DIFF_THRESH) & ~shadow)
    weak = chromaFg | (dsum >= DIFF_THRESH)
    return strong, weak


def hue_class(r, g, b):
    mx, mn = max(r, g, b), min(r, g, b)
    d = mx - mn
    if d < PX_CHROMA_MIN or d < mx * PX_SAT_MIN:
        return 0
    if mx == r:
        h = 60 * math.fmod((g - b) / d, 6)
    elif mx == g:
        h = 60 * ((b - r) / d + 2)
    else:
        h = 60 * ((r - g) / d + 4)
    h += 360 if h < 0 else 0
    if h >= HUE[0] or h < HUE[1]:
        return 3
    if h < HUE[2]:
        return 4
    if h < HUE[3]:
        return 5
    if h < HUE[4]:
        return 6
    return 7


def classify_color(raw, blob, gain, plate_gray, wb_split):
    p = np.pad(blob, 1)
    interior = blob & p[:-2, 1:-1] & p[2:, 1:-1] & p[1:-1, :-2] & p[1:-1, 2:]
    use = blob if interior.sum() < 20 else interior
    votes = [0] * 8
    vsum, vn = 0.0, 0
    for y, x in zip(*np.nonzero(use)):
        r, g, b = (min(255.0, raw[y, x, k] * gain[k]) for k in range(3))
        c = hue_class(r, g, b)
        if c == 0:
            v = max(r, g, b)
            c = 1 if v >= plate_gray * wb_split else 2
            vsum += v
            vn += 1
        votes[c] += 1
    n = sum(votes)
    share = sum(votes[3:]) / n if n else 0
    vratio = vsum / vn / plate_gray if vn else 0
    if share >= CHROMA_SHARE_MIN:
        best = max(range(3, 8), key=lambda c: votes[c])
        return (0 if best == 7 else best), vratio
    return (1 if vratio >= wb_split else 2), vratio


def grow_black(blob, weak_maj, below):
    """검정 물체: 그림자 필터로 빠진 픽셀 중 '같은 열(±BLACK_GROW_COLS)에서 덩어리 바닥 + below 보다 위'인 것만 붙인다.
    반사로 회색이 된 윗면과, 빛을 받아 회색으로 보이는 카메라 쪽 옆면(below px)까지 되살리고, 그보다 아래 그림자는 붙이지 않는다."""
    H, W = blob.shape
    bot = np.full(W, -1)
    for x in np.nonzero(blob.any(0))[0]:
        bot[x] = np.nonzero(blob[:, x])[0].max()
    wb = np.array([bot[max(0, x - BLACK_GROW_COLS):x + BLACK_GROW_COLS + 1].max() for x in range(W)])
    allow = (np.arange(H)[:, None] <= wb[None, :] + below) & (wb[None, :] >= 0)
    return flood(weak_maj | blob, list(zip(*np.nonzero(blob))), allow | blob)


def opening(b):
    H, W = b.shape
    p = np.pad(b, 1)
    er = np.ones_like(b)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            er &= p[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
    p = np.pad(er, 1)
    di = np.zeros_like(b)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            di |= p[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
    return di & b


def convex_hull(pts):
    """정수 좌표 점들의 볼록 껍질 (모노톤 체인, 반시계)"""
    pts = sorted(set(pts))
    if len(pts) < 3:
        return pts
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def mask_hull(mask, x0, y0):
    """측정 마스크 가장자리의 볼록 껍질 (전체 영상 좌표). 외접 사각형은 볼록 껍질 꼭짓점만 보면 된다"""
    p = np.pad(mask, 1)
    edge = mask & ~(p[:-2, 1:-1] & p[2:, 1:-1] & p[1:-1, :-2] & p[1:-1, 2:])
    ys, xs = np.nonzero(edge)
    return convex_hull([(int(x) + x0, int(y) + y0) for x, y in zip(xs, ys)])


def side_for(color, prof):
    return prof['sideW'] if color == 1 else (prof['sideK'] if color == 2 else prof['sideC'])


def measure_hull(hull, side, P, K, px35):
    """원근 보정 좌표에서 옆면(아래쪽 side만큼)을 뺀 최소 외접 사각형 → (길이mm, 폭mm)"""
    if len(hull) < 3:
        return 0.0, 0.0
    h = np.asarray(hull, dtype=float)
    dv = h[:, 1] - IMG_CY
    g = 1.0 / (1.0 + P * dv)
    X = (h[:, 0] - IMG_CX) * g
    Y = K * dv * g
    hx, hy = 0.5 * g, 0.5 * K * g * g
    best = None
    for deg in range(0, 90, CALIPER_STEP_DEG):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        pu, pv = X * c + Y * s, -X * s + Y * c
        ru, rv = hx * c + hy * s, hx * s + hy * c
        lu = max(1.0, (pu + ru).max() - (pu - ru).min() - side * s)
        lv = max(1.0, (pv + rv).max() - (pv - rv).min() - side * c)
        if best is None or lu * lv < best[0]:
            best = (lu * lv, lu, lv)
    _, lu, lv = best
    return max(lu, lv) * WIDTH_MM / px35, min(lu, lv) * WIDTH_MM / px35


def size_of(len_mm, prof):
    if len_mm <= 0:
        return 0
    return 1 if len_mm < prof['smallMax'] else 2 if len_mm < prof['mediumMax'] else 3


def analyze(t, prof):
    """샘플 한 장을 펌웨어와 같은 방법으로 분할·색 판정·측정"""
    s, raw, bg = t['s'], t['raw'], t['bg']
    strong, weak = foreground(raw, bg, s['gain'])
    blob = largest(majority(strong))
    plate_gray = sum(s['plate']) / 3
    color, vratio = classify_color(raw, blob, s['gain'], plate_gray, prof['wbSplit'])
    mask, grown = blob, 0
    if color == 2:
        mask = grow_black(blob, majority(weak), prof['below'])
        grown = int((mask & ~blob).sum())   # 펌웨어 BlobInfo.grown 과 같은 값 (열림 전에 붙인 픽셀 수)
    mask = largest(opening(mask))     # 가는 선·떨어진 점 제거 후 가장 큰 덩어리만
    hull = mask_hull(mask, t['x0'], t['y0'])
    L, Wd = measure_hull(hull, side_for(color, prof), prof['P'], prof['K'], prof['px35'])
    return dict(mask=mask, hull=hull, color=color, vratio=vratio, grown=grown, L=L, W=Wd)


# ---------- 기준 계산 (펌웨어 fitProfile() 과 같은 방법) ----------
FIT_DELTA = 0.08                      # 의사-Huber 손실의 꺾이는 점 (길이·폭의 로그 오차 약 8%)
FIT_LO = (-0.01, 0.8, 15.0, 0.0, 0.0, 0.0)
FIT_HI = (0.02, 1.6, 60.0, 30.0, 30.0, 30.0)
FIT_STEP = (0.002, 0.05, 2.0, 3.0, 3.0, 3.0)
FIT_MAX_EVALS = 700


def fit_cost(x, data):
    """data: [(hull, colorGroup 0/1/2, sizeLabel)] — 길이는 명목값, 폭은 35mm가 되도록"""
    for v, lo, hi in zip(x, FIT_LO, FIT_HI):
        if v < lo or v > hi:
            return 1e9
    P, K, px35 = x[0], x[1], x[2]
    e = 0.0
    for hull, grp, size in data:
        L, Wd = measure_hull(hull, x[3 + grp], P, K, px35)
        if L <= 0 or Wd <= 0:
            return 1e9
        for r in (math.log(L / NOMINAL_MM[size]), math.log(Wd / WIDTH_MM)):
            e += FIT_DELTA * FIT_DELTA * (math.sqrt(1 + (r / FIT_DELTA) ** 2) - 1)
    return e


def nelder_mead(f, x0, steps, max_evals):
    """표준 Nelder-Mead (반사 1, 확장 2, 수축 0.5, 축소 0.5) — 펌웨어 nelderMead() 와 같은 순서"""
    n = len(x0)
    simplex = [list(x0)] + [[x0[j] + (steps[j] if j == i else 0) for j in range(n)] for i in range(n)]
    fv = [f(p) for p in simplex]
    evals = n + 1
    while evals < max_evals:
        order = sorted(range(n + 1), key=lambda i: fv[i])
        simplex = [simplex[i] for i in order]
        fv = [fv[i] for i in order]
        if fv[n] - fv[0] < 1e-7:
            break
        c = [sum(simplex[i][j] for i in range(n)) / n for j in range(n)]
        w = simplex[n]
        xr = [c[j] + (c[j] - w[j]) for j in range(n)]
        fr = f(xr); evals += 1
        if fr < fv[0]:
            xe = [c[j] + 2 * (c[j] - w[j]) for j in range(n)]
            fe = f(xe); evals += 1
            simplex[n], fv[n] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < fv[n - 1]:
            simplex[n], fv[n] = xr, fr
        else:
            if fr < fv[n]:
                xc = [c[j] + 0.5 * (xr[j] - c[j]) for j in range(n)]
            else:
                xc = [c[j] + 0.5 * (w[j] - c[j]) for j in range(n)]
            fc = f(xc); evals += 1
            if fc < min(fr, fv[n]):
                simplex[n], fv[n] = xc, fc
            else:
                for i in range(1, n + 1):
                    simplex[i] = [simplex[0][j] + 0.5 * (simplex[i][j] - simplex[0][j]) for j in range(n)]
                    fv[i] = f(simplex[i]); evals += 1
    k = min(range(n + 1), key=lambda i: fv[i])
    return simplex[k], fv[k], evals


def fit_profile(items, res, base):
    """라벨이 있는 샘플로 새 기준 계산. 반환: (새 프로필, 정보)"""
    data = [(r['hull'], 0 if r['color'] == 1 else (1 if r['color'] == 2 else 2), t['s']['size'])
            for t, r in zip(items, res) if t['s']['size'] in NOMINAL_MM and len(r['hull']) >= 3]
    f = lambda x: fit_cost(x, data)
    x0 = [base['P'], base['K'], base['px35'], base['sideW'], base['sideK'], base['sideC']]
    x, fx, ev1 = nelder_mead(f, x0, FIT_STEP, FIT_MAX_EVALS)
    x, fx, ev2 = nelder_mead(f, x, FIT_STEP, FIT_MAX_EVALS)        # 한 번 더 (국소 정체 벗어나기)
    prof = dict(base, name='현장 학습 (계산)', P=x[0], K=x[1], px35=x[2], sideW=x[3], sideK=x[4], sideC=x[5])
    # 크기 경계: 이웃한 크기의 측정 범위가 나뉘면 그 사이(기하 평균), 겹치면 명목 길이 사이
    by = {1: [], 2: [], 3: []}
    for hull, grp, size in data:
        by[size].append(measure_hull(hull, x[3 + grp], x[0], x[1], x[2])[0])
    def bound(a, b):
        if by[a] and by[b] and max(by[a]) < min(by[b]):
            return math.sqrt(max(by[a]) * min(by[b]))
        return math.sqrt(NOMINAL_MM[a] * NOMINAL_MM[b])
    prof['smallMax'], prof['mediumMax'] = bound(1, 2), bound(2, 3)
    # 흰/검 경계: 두 무리의 밝기비가 나뉘면 그 사이
    vw = [r['vratio'] for t, r in zip(items, res) if t['s']['color'] == 1 and r['vratio'] > 0]
    vk = [r['vratio'] for t, r in zip(items, res) if t['s']['color'] == 2 and r['vratio'] > 0]
    if vw and vk and min(vw) > max(vk):
        prof['wbSplit'] = math.sqrt(min(vw) * max(vk))
    return prof, dict(cost=fx, evals=ev1 + ev2, n=len(data), ranges={k: (min(v), max(v)) if v else None for k, v in by.items()})


def report(items, res, prof):
    ok_s = ok_c = n_s = n_c = 0
    by = {1: [], 2: [], 3: []}
    print(' id 정답  판별  길이mm  폭mm  색(정답→판별) 밝기비 보강px  샘플 찍을 때 판별')
    for t, r in zip(items, res):
        s = t['s']
        sz = size_of(r['L'], prof)
        if s['size']:
            n_s += 1
            ok_s += sz == s['size']
            by[s['size']].append(r['L'])
        if s['color']:
            n_c += 1
            ok_c += r['color'] == s['color']
        mark = '' if sz == s['size'] or not s['size'] else '  ← 크기 틀림'
        mark += '' if r['color'] == s['color'] or not s['color'] else '  ← 색 틀림'
        print(f"{s['id']:3}  {SIZE_NAMES[s['size']]:3}  {SIZE_NAMES[sz]:3}  {r['L']:6.1f} {r['W']:5.1f}   "
              f"{COLOR_NAMES[s['color']]}→{COLOR_NAMES[r['color']]}     {r['vratio']:4.2f} {r['grown']:5}   "
              f"{SIZE_NAMES[s.get('predSize', 0)]}/{COLOR_NAMES[s.get('predColor', 0)]}{mark}")
    print(f"\n[{prof['name']}] 크기 정답 {ok_s}/{n_s}, 색상 정답 {ok_c}/{n_c}")
    old_s = sum(1 for t in items if t['s']['size'] and t['s'].get('predSize') == t['s']['size'])
    old_c = sum(1 for t in items if t['s']['color'] and t['s'].get('predColor') == t['s']['color'])
    print(f"(샘플을 찍을 당시 장치 판별: 크기 {old_s}/{n_s}, 색상 {old_c}/{n_c})")
    print('크기별 측정 길이 범위 (경계: 소/중 %.1fmm, 중/대 %.1fmm)' % (prof['smallMax'], prof['mediumMax']))
    for c, v in by.items():
        if v:
            print(f"  {SIZE_NAMES[c]}: {min(v):5.1f} ~ {max(v):5.1f} mm  ({len(v)}개)")
    return ok_s, n_s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('zip')
    ap.add_argument('--profile', type=int, default=DEFAULT_PROFILE, choices=sorted(PROFILES), help='판별 기준 번호 (기본 2)')
    ap.add_argument('--fit', action='store_true', help='샘플로 새 기준 계산 (시작값은 --profile)')
    a = ap.parse_args()
    meta, items = load_zip(a.zip)
    prof = PROFILES[a.profile]
    print(f"샘플 {len(items)}개 (펌웨어 {meta.get('fw')}, 장치 {meta.get('host')}), 기준: {prof['name']}\n")
    res = [analyze(t, prof) for t in items]
    report(items, res, prof)
    if a.fit:
        new, info = fit_profile(items, res, prof)
        res2 = [dict(r, **dict(zip(('L', 'W'), measure_hull(r['hull'], side_for(r['color'], new), new['P'], new['K'], new['px35']))))
                for r in res]
        print(f"\n===== 새 기준 (샘플 {info['n']}개, 비용 {info['cost']:.4f}, 계산 {info['evals']}회) =====")
        report(items, res2, new)
        print('\n펌웨어 PROFILE_BUILTIN 에 넣을 값:')
        print('  {"이름", %.5ff, %.4ff, %.2ff, %.2ff, %.2ff, %.2ff, %d, %.1ff, %.1ff, %.2ff},' % (
            new['P'], new['K'], new['px35'], new['sideW'], new['sideK'], new['sideC'], new['below'],
            new['smallMax'], new['mediumMax'], new['wbSplit']))


if __name__ == '__main__':
    main()
