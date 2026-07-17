"""
Công cụ chuẩn đoán chi tiết từng nhánh vẽ (Debug Branches Diagnostics) cho 3 Thí nghiệm.
Cho phép kiểm tra chỉ số hình học (arc, chord, ratio, width) theo từng Tập hoặc so sánh cả 3 Tập.
Cú pháp:
  python debug_branches.py          -> Kiểm tra và in chi tiết nhánh của cả 3 Tập (Tập 1, Tập 2, Tập 3)
  python debug_branches.py "Tập 2"  -> Chỉ kiểm tra riêng Tập 2
  python debug_branches.py compare  -> Bảng so sánh nhanh số nhánh & độ dài nhánh giữa 3 Tập
"""
import sys, math
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
import centerline_extractor as ce

NAMES = ['letter_H', 'letter_K', 'number_3', 'number_6', 'ampersand', 'arrow-pointer', 'arrow-turn-down-left']

def debug_run(run_choice):
    ce.EXPERIMENTAL_RUN = run_choice
    print(f"\n{'='*85}")
    print(f"  [>>> DIAGNOSTICS CHI TIẾT NHÁNH VẼ CHIẾN LƯỢC: {run_choice} <<<]")
    print(f"{'='*85}")

    for name in NAMES:
        png_path = f'challenge_sample/{name}.png'
        matrix, w, h = ce.PurePNG.read_binary_matrix(png_path)
        contours = ce.BoundaryDetector.find_all_contours(matrix, w, h)
        midpoints = ce.RayCasterDDA.cast_rays(matrix, contours, w, h)
        avg_w = sum(m[2] for m in midpoints) / len(midpoints) if midpoints else 0
        branches = ce.TopologyResolver.build_graph_and_segment(midpoints)
        print(f"\n▶ {name:22s} ({len(branches):2d} nhánh | avg_w={avg_w:.1f}px):")
        for i, b in enumerate(branches):
            arc = sum(math.hypot(b[j+1][0]-b[j][0], b[j+1][1]-b[j][1]) for j in range(len(b)-1))
            chord = math.hypot(b[0][0]-b[-1][0], b[0][1]-b[-1][1])
            ratio = chord/arc if arc > 0 else 0
            w_start = b[0][2] if len(b[0])>=3 else avg_w
            w_end = b[-1][2] if len(b[-1])>=3 else avg_w
            marker = ' <<< SHORT' if arc < avg_w else ''
            if w_start < 0.35 * avg_w or w_end < 0.35 * avg_w:
                marker += ' [SHARP TIP]'
            print(f'  b{i+1:2d}: {len(b):3d} pts | arc={arc:6.1f} chord={chord:6.1f} ratio={ratio:.2f} (w: {w_start:5.1f}->{w_end:5.1f}){marker}')

def compare_runs_table():
    print(f"\n{'='*95}")
    print(f"  BẢNG SO SÁNH NHANH SỐ LƯỢNG VÀ ĐẶC ĐIỂM NHÁNH TRÊN CẢ 3 TẬP")
    print(f"{'='*95}")
    print(f"{'Hình mẫu':22s} | {'Tập 1 (Nhánh / Chóp)':22s} | {'Tập 2 (Nhánh / Chóp)':22s} | {'Tập 3 (Nhánh / Chóp)':22s}")
    print('-' * 95)
    for name in NAMES:
        png_path = f'challenge_sample/{name}.png'
        matrix, w, h = ce.PurePNG.read_binary_matrix(png_path)
        contours = ce.BoundaryDetector.find_all_contours(matrix, w, h)
        row = f"{name:22s}"
        for run_choice in ["Tập 1", "Tập 2", "Tập 3"]:
            ce.EXPERIMENTAL_RUN = run_choice
            midpoints = ce.RayCasterDDA.cast_rays(matrix, contours, w, h)
            avg_w = sum(m[2] for m in midpoints) / len(midpoints) if midpoints else 100.0
            branches = ce.TopologyResolver.build_graph_and_segment(midpoints)
            tips = sum(1 for b in branches if (len(b[0])>=3 and b[0][2]<0.35*avg_w) or (len(b[-1])>=3 and b[-1][2]<0.35*avg_w))
            row += f" | {len(branches):6d} nhánh ({tips:2d} chóp)"
        print(row)
    print(f"{'='*95}\n")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        arg = sys.argv[1].strip()
        if arg in ["Tập 1", "Tập 2", "Tập 3"]:
            debug_run(arg)
        elif arg.lower() in ["compare", "table", "all_table"]:
            compare_runs_table()
        elif arg.lower() in ["all", "cả 3", "3"]:
            for r in ["Tập 1", "Tập 2", "Tập 3"]: debug_run(r)
        else:
            print(f"Tùy chọn không hợp lệ: {arg}. Chạy mặc định cho cả 3 Tập.")
            for r in ["Tập 1", "Tập 2", "Tập 3"]: debug_run(r)
    else:
        # Mặc định khi chạy python debug_branches.py sẽ in chi tiết cả 3 Tập và bảng so sánh
        compare_runs_table()
        for r in ["Tập 1", "Tập 2", "Tập 3"]:
            debug_run(r)
