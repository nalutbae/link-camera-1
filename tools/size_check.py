"""학습 샘플(samples.zip)로 크기·색상 판별을 검증하고 보정값을 다시 맞추는 도구.

펌웨어(src/main.cpp)의 판별 과정을 그대로 재현한다:
  1) 배경 차분 → 강한 전경(그림자 필터 적용) / 약한 전경(그림자 필터 없음)
  2) 3x3 다수결 → 가장 큰 덩어리 → 색상 판정
  3) 검정 물체면 약한 전경으로 덩어리를 키움 (반사로 회색이 된 윗면 복원)
  4) 3x3 열림 → 원근 보정 좌표에서 옆면을 뺀 최소 외접 사각형 → 길이(mm)

사용법:
  python tools/size_check.py link-cam1_samples.zip            # 현재 상수로 판별 결과 보기
  python tools/size_check.py link-cam1_samples.zip --fit      # 흰색 샘플로 원근 보정값 다시 맞추기
필요 패키지: numpy, pillow (--fit 은 scipy 도 필요)
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
WHITE_BLACK_SPLIT = 1.2
HUE = (335.0, 28.0, 68.0, 165.0, 260.0)      # 빨강 시작, 노랑, 초록, 파랑, 파랑 끝
BLACK_GROW_COLS = 8
SIZE_PERSP_P = 0.00203
SIZE_Y_SCALE = 1.1432
SIZE_PX_PER_35MM = 31.48
SIDE_FACE_PX = 10.0
SIZE_SMALL_MAX_MM, SIZE_MEDIUM_MAX_MM = 47.0, 58.5
CALIPER_STEP_DEG = 3
IMG_CX, IMG_CY = 80.0, 60.0                   # 원근 보정 기준점 (전체 160x120 영상의 중심)

SIZE_NAMES = ['미판별', '소', '중', '대']
COLOR_NAMES = ['미판별', '흰', '검', '빨', '노', '초', '파']
NOMINAL_MM = {1: 35, 2: 55, 3: 85}


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


def classify_color(raw, blob, gain, plate_gray):
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
            c = 1 if v >= plate_gray * WHITE_BLACK_SPLIT else 2
            vsum += v
            vn += 1
        votes[c] += 1
    n = sum(votes)
    share = sum(votes[3:]) / n if n else 0
    vratio = vsum / vn / plate_gray if vn else 0
    if share >= CHROMA_SHARE_MIN:
        best = max(range(3, 8), key=lambda c: votes[c])
        return (0 if best == 7 else best), vratio
    return (1 if vratio >= WHITE_BLACK_SPLIT else 2), vratio


def grow_black(blob, weak_maj):
    """검정 물체: 그림자 필터로 빠진 픽셀 중 '같은 열(±BLACK_GROW_COLS)에서 덩어리 바닥보다 위'인 것만 덩어리에 붙인다.
    카메라 쪽 옆면은 빛을 못 받아 항상 진하게 잡히므로, 그림자(옆면보다 더 아래)는 들어오지 않는다."""
    H, W = blob.shape
    bot = np.full(W, -1)
    for x in np.nonzero(blob.any(0))[0]:
        bot[x] = np.nonzero(blob[:, x])[0].max()
    wb = np.array([bot[max(0, x - BLACK_GROW_COLS):x + BLACK_GROW_COLS + 1].max() for x in range(W)])
    allow = (np.arange(H)[:, None] <= wb[None, :]) & (wb[None, :] >= 0)
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


def measure(mask, x0, y0, side, P=None, K=None, px35=None):
    """원근 보정 좌표에서 옆면(아래쪽 side만큼)을 뺀 최소 외접 사각형 → (길이mm, 폭mm)"""
    P = SIZE_PERSP_P if P is None else P
    K = SIZE_Y_SCALE if K is None else K
    px35 = SIZE_PX_PER_35MM if px35 is None else px35
    p = np.pad(mask, 1)
    edge = mask & ~(p[:-2, 1:-1] & p[2:, 1:-1] & p[1:-1, :-2] & p[1:-1, 2:])
    ys, xs = np.nonzero(edge)
    if len(xs) < 3:
        return 0.0, 0.0
    dv = ys + y0 - IMG_CY
    g = 1.0 / (1.0 + P * dv)
    X = (xs + x0 - IMG_CX) * g
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
    return max(lu, lv) * 35 / px35, min(lu, lv) * 35 / px35


def size_of(len_mm):
    if len_mm <= 0:
        return 0
    return 1 if len_mm < SIZE_SMALL_MAX_MM else 2 if len_mm < SIZE_MEDIUM_MAX_MM else 3


def analyze(t):
    s, raw, bg = t['s'], t['raw'], t['bg']
    strong, weak = foreground(raw, bg, s['gain'])
    blob = largest(majority(strong))
    plate_gray = sum(s['plate']) / 3
    color, vratio = classify_color(raw, blob, s['gain'], plate_gray)
    mask, grown = blob, 0
    if color == 2:
        mask = grow_black(blob, majority(weak))
        grown = int((mask & ~blob).sum())   # 펌웨어 BlobInfo.grown 과 같은 값 (열림 전에 붙인 픽셀 수)
    mask = largest(opening(mask))     # 가는 선·떨어진 점 제거 후 가장 큰 덩어리만
    side = 0.0 if color == 1 else SIDE_FACE_PX
    return dict(mask=mask, color=color, vratio=vratio, side=side, grown=grown)


def fit_perspective(items):
    """흰색 샘플(윗면만 잡힘)로 원근 P, 세로 축척 K, 35mm 폭(px)을 맞춘다"""
    from scipy.optimize import minimize
    white = [(t, a) for t, a in items if t['s']['color'] == 1 and t['s']['size'] in NOMINAL_MM]
    if len(white) < 4:
        sys.exit('흰색 샘플이 4개 이상 필요합니다')

    def cost(th):
        P, K = th
        e = []
        for t, a in white:
            L, Wd = measure(a['mask'], t['x0'], t['y0'], 0, P, K, 35.0)   # 35mm = 35 단위로 두고 비율만 본다
            e.append((math.log(L / Wd) - math.log(NOMINAL_MM[t['s']['size']] / 35)) ** 2)
            e.append(0.25 * math.log(Wd) ** 2)
        return sum(e[0::2]) + np.var(e[1::2])
    r = minimize(cost, [SIZE_PERSP_P, SIZE_Y_SCALE], method='Nelder-Mead', options=dict(xatol=1e-5, fatol=1e-8))
    P, K = r.x
    ws = [measure(a['mask'], t['x0'], t['y0'], 0, P, K, 35.0)[1] for t, a in white]
    return P, K, float(np.median(ws))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('zip')
    ap.add_argument('--fit', action='store_true', help='흰색 샘플로 원근 보정값(SIZE_PERSP_P, SIZE_Y_SCALE, SIZE_PX_PER_35MM) 다시 맞추기')
    a = ap.parse_args()
    meta, items = load_zip(a.zip)
    print(f"샘플 {len(items)}개 (펌웨어 {meta.get('fw')}, 장치 {meta.get('host')})\n")
    res = [(t, analyze(t)) for t in items]

    if a.fit:
        P, K, px35 = fit_perspective(res)
        print(f"보정값 제안: SIZE_PERSP_P = {P:.5f}f, SIZE_Y_SCALE = {K:.4f}f, SIZE_PX_PER_35MM = {px35:.2f}f\n")
        global SIZE_PERSP_P, SIZE_Y_SCALE, SIZE_PX_PER_35MM
        SIZE_PERSP_P, SIZE_Y_SCALE, SIZE_PX_PER_35MM = P, K, px35

    print(' id 정답  판별  길이mm  폭mm  색(정답→판별) 밝기비 보강px  예전판별')
    ok_s = ok_c = n_s = n_c = 0
    by_size = {1: [], 2: [], 3: []}
    for t, r in res:
        s = t['s']
        L, Wd = measure(r['mask'], t['x0'], t['y0'], r['side'])
        sz = size_of(L)
        if s['size']:
            n_s += 1
            ok_s += sz == s['size']
            by_size[s['size']].append(L)
        if s['color']:
            n_c += 1
            ok_c += r['color'] == s['color']
        mark = '' if sz == s['size'] or not s['size'] else '  ← 크기 틀림'
        mark += '' if r['color'] == s['color'] or not s['color'] else '  ← 색 틀림'
        print(f"{s['id']:3}  {SIZE_NAMES[s['size']]:3}  {SIZE_NAMES[sz]:3}  {L:6.1f} {Wd:5.1f}   "
              f"{COLOR_NAMES[s['color']]}→{COLOR_NAMES[r['color']]}     {r['vratio']:4.2f} {r['grown']:5}   "
              f"{SIZE_NAMES[s.get('predSize', 0)]}/{COLOR_NAMES[s.get('predColor', 0)]}{mark}")
    print(f"\n크기 정답 {ok_s}/{n_s}, 색상 정답 {ok_c}/{n_c}")
    old_s = sum(1 for t, _ in res if t['s']['size'] and t['s'].get('predSize') == t['s']['size'])
    old_c = sum(1 for t, _ in res if t['s']['color'] and t['s'].get('predColor') == t['s']['color'])
    print(f"(샘플을 찍을 당시 장치 판별: 크기 {old_s}/{n_s}, 색상 {old_c}/{n_c})")
    print('\n크기별 측정 길이 범위 (경계: 소/중 %.1fmm, 중/대 %.1fmm)' % (SIZE_SMALL_MAX_MM, SIZE_MEDIUM_MAX_MM))
    for c, v in by_size.items():
        if v:
            print(f"  {SIZE_NAMES[c]}: {min(v):5.1f} ~ {max(v):5.1f} mm  ({len(v)}개)")
    if all(by_size.values()):
        # 이웃한 두 크기의 측정 범위 사이 (기하 평균) — 겹치면 0 이하의 간격이 표시된다
        b1 = math.sqrt(max(by_size[1]) * min(by_size[2]))
        b2 = math.sqrt(max(by_size[2]) * min(by_size[3]))
        print(f"경계 제안: SIZE_SMALL_MAX_MM = {b1:.1f}f (간격 {min(by_size[2]) - max(by_size[1]):+.1f}mm), "
              f"SIZE_MEDIUM_MAX_MM = {b2:.1f}f (간격 {min(by_size[3]) - max(by_size[2]):+.1f}mm)")


if __name__ == '__main__':
    main()
