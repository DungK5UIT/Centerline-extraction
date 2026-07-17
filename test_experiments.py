"""
Bộ công cụ kiểm tra tự động (Unified Test Harness) cho 3 Thí nghiệm trong centerline_extractor.py.
Chạy lần lượt Tập 1, Tập 2, Tập 3 và lưu kết quả vào output_tap1, output_tap2, output_tap3.
"""
import os, glob, time, sys
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
import centerline_extractor as ce

SAMPLE_DIR = "challenge_sample"
RUNS = [
    ("Tập 1", "output_tap1", "Longitudinal Ray Warp + Angle-Weighted Clustering"),
    ("Tập 2", "output_tap2", "Sub-Pixel Corner Snap + Quadratic Bézier Transition"),
    ("Tập 3", "output_tap3", "Spline Extrapolation & Handle Clamping + Curvature RDP")
]

def run_all_experiments():
    if not os.path.exists(SAMPLE_DIR):
        print(f"Lỗi: Không tìm thấy thư mục ảnh mẫu '{SAMPLE_DIR}'!")
        return

    png_files = sorted(glob.glob(os.path.join(SAMPLE_DIR, "*.png")))
    print(f"\n{'='*75}")
    print(f"  BẮT ĐẦU CHẠY THÍ NGHIỆM TỔNG HỢP ({len(png_files)} ẢNH MẪU / 3 TẬP)")
    print(f"{'='*75}")

    summary = {}
    for run_name, out_dir, desc in RUNS:
        os.makedirs(out_dir, exist_ok=True)
        ce.EXPERIMENTAL_RUN = run_name
        print(f"\n▶ [{run_name}] {desc}")
        print(f"  Thư mục xuất: {out_dir}")
        
        start_t = time.time()
        results = {}
        for i, png_file in enumerate(png_files, 1):
            base_name = os.path.splitext(os.path.basename(png_file))[0]
            svg_path = os.path.join(out_dir, f"{base_name}.svg")
            try:
                curves = ce.process_image(png_file, svg_path)
                results[base_name] = curves
                print(f"    ({i}/{len(png_files)}) {base_name:22s} -> {curves:2d} nét vẽ ✓")
            except Exception as e:
                results[base_name] = -1
                print(f"    ({i}/{len(png_files)}) {base_name:22s} -> LỖI: {e}")
        
        elapsed = time.time() - start_t
        summary[run_name] = (results, elapsed)

    print(f"\n{'='*75}")
    print(f"  BẢNG TỔNG HỢP KẾT QUẢ SỐ NÉT VẼ TRÊN 3 THÍ NGHIỆM")
    print(f"{'='*75}")
    print(f"{'Tên hình mẫu':25s} | {'Tập 1':>8s} | {'Tập 2':>8s} | {'Tập 3':>8s}")
    print('-' * 57)
    
    all_names = sorted([os.path.splitext(os.path.basename(p))[0] for p in png_files])
    for name in all_names:
        c1 = summary["Tập 1"][0].get(name, -1)
        c2 = summary["Tập 2"][0].get(name, -1)
        c3 = summary["Tập 3"][0].get(name, -1)
        print(f"{name:25s} | {c1:8d} | {c2:8d} | {c3:8d}")
    
    print('-' * 57)
    print(f"Thời gian chạy: Tập 1={summary['Tập 1'][1]:.1f}s | Tập 2={summary['Tập 2'][1]:.1f}s | Tập 3={summary['Tập 3'][1]:.1f}s")
    print(f"{'='*75}\n")

if __name__ == "__main__":
    run_all_experiments()
