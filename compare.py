"""
So sánh số lượng nét vẽ (SVG Paths) giữa các bộ kết quả của chúng ta và bộ chuẩn Reference.
"""
import xml.etree.ElementTree as ET
import os, sys

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')


REF_DIR = 'challenge_sample_results'
OUR_DIRS = [
    ('Mặc định (challenge_sample_output)', 'challenge_sample_output'),
    ('Tập 1 (output_tap1)', 'output_tap1'),
    ('Tập 2 (output_tap2)', 'output_tap2'),
    ('Tập 3 (output_tap3)', 'output_tap3'),
]

def count_svg_paths(filepath, exclude_id_prefix=None):
    if not os.path.exists(filepath): return None
    try:
        tree = ET.parse(filepath)
        root = tree.getroot()
        ns = root.tag[root.tag.index('{'):root.tag.index('}')+1] if '{' in root.tag else ''
        paths = root.findall(f'.//{ns}path')
        if exclude_id_prefix:
            paths = [p for p in paths if not p.get('id','').startswith(exclude_id_prefix)]
        return len(paths)
    except Exception:
        return None

def main():
    if not os.path.exists(REF_DIR):
        print(f"Không tìm thấy thư mục chuẩn '{REF_DIR}'!")
        return

    ref_counts = {}
    for fname in sorted(os.listdir(REF_DIR)):
        if not fname.endswith('.svg'): continue
        # Tham chiếu trong challenge_sample_results thường có path-10... là nền hoặc khung rác nếu có
        cnt = count_svg_paths(os.path.join(REF_DIR, fname), exclude_id_prefix='path-10')
        ref_counts[fname] = cnt

    # Chỉ so sánh các folder đang tồn tại trong workspace
    active_dirs = [(name, d) for name, d in OUR_DIRS if os.path.exists(d)]
    
    print(f"\n{'='*95}")
    print(f"  BẢNG SO SÁNH SỐ LƯỢNG NÉT VẼ SO VỚI REFERENCE ({REF_DIR})")
    print(f"{'='*95}")
    
    header = f"{'File hình mẫu':25s} | {'Reference':>9s}"
    for name, d in active_dirs:
        header += f" | {d:>12s} (Δ)"
    print(header)
    print('-' * len(header))

    for fname, r_cnt in sorted(ref_counts.items()):
        r_str = str(r_cnt) if r_cnt is not None else 'N/A'
        row = f"{fname:25s} | {r_str:>9s}"
        for name, d in active_dirs:
            o_cnt = count_svg_paths(os.path.join(d, fname))
            if o_cnt is None:
                row += f" | {'N/A':>16s}"
            else:
                delta = o_cnt - r_cnt if r_cnt is not None else 0
                row += f" | {o_cnt:7d} ({delta:+3d})"
        print(row)
    print(f"{'='*95}\n")

if __name__ == "__main__":
    main()
