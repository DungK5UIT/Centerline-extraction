# =========================================================================
# CẤU HÌNH THÍ NGHIỆM CHIẾN LƯỢC HÌNH HỌC (UNIFIED CONFIGURATION SWITCH)
# =========================================================================
# Các tùy chọn chiến lược R&D dành cho nêm nhọn mũi tên (Arrowhead Wedge):
# - "Tập 1": Longitudinal Ray Warp (Xoay tia DDA theo chiều dọc góc chóp) + Angle-Weighted Clustering
# - "Tập 2": Sub-Pixel Corner Snap (Phát hiện góc < 50°, lọc DDA 1.5*avg_w & snap trục trung vị bisector)
# - "Tập 3": Spline Extrapolation & Handle Clamping (Ngoại suy chóp mượt 0.5*w & ép tay quay Bézier = 0)
EXPERIMENTAL_RUN = "Tập 2"

import os, sys, glob, math, zlib, struct
from collections import defaultdict
import xml.etree.ElementTree as ET

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')


# =========================================================================
# BƯỚC 1: ĐỌC VÀ GIẢI MÃ ẢNH PNG THÔ (DATA INGESTION)
# =========================================================================
class PurePNG:
    """Stage 1: Đọc nhị phân, giải nén zlib và chuyển về ma trận nhị phân 0-1 không dùng thư viện ngoài."""
    @staticmethod
    def paeth(a, b, c):
        p = a + b - c
        pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
        return a if pa <= pb and pa <= pc else (b if pb <= pc else c)

    @classmethod
    def read_binary_matrix(cls, filepath, threshold=127):
        with open(filepath, 'rb') as f:
            if f.read(8) != b'\x89PNG\r\n\x1a\n': raise ValueError("PNG không hợp lệ!")
            chunks = []
            while True:
                length = struct.unpack('>I', f.read(4))[0]
                ctype, cdata = f.read(4), f.read(length); f.read(4)
                chunks.append((ctype, cdata))
                if ctype == b'IEND': break

        w, h, depth, ctype = struct.unpack('>IIBB', chunks[0][1][:10])
        decomp = zlib.decompress(b"".join(d for t, d in chunks if t == b'IDAT'))
        bpp = 4 if ctype == 6 else (3 if ctype == 2 else 1)
        stride, matrix, recon, prev = w * bpp, [], bytearray(w * bpp), bytearray(w * bpp)
        
        idx = 0
        for _ in range(h):
            ftype, row, idx = decomp[idx], decomp[idx+1 : idx+1+stride], idx+1+stride
            for i in range(stride):
                raw, a, b, c = row[i], (recon[i-bpp] if i>=bpp else 0), prev[i], (prev[i-bpp] if i>=bpp else 0)
                if ftype == 0: recon[i] = raw
                elif ftype == 1: recon[i] = (raw + a) & 0xFF
                elif ftype == 2: recon[i] = (raw + b) & 0xFF
                elif ftype == 3: recon[i] = (raw + (a + b) // 2) & 0xFF
                elif ftype == 4: recon[i] = (raw + cls.paeth(a, b, c)) & 0xFF
            prev = bytearray(recon)
            row_px = [1 if (r+g+b)<3*threshold and a>threshold else 0 for r,g,b,a in zip(recon[0::4], recon[1::4], recon[2::4], recon[3::4])] if ctype==6 else \
                     ([1 if (r+g+b)<3*threshold else 0 for r,g,b in zip(recon[0::3], recon[1::3], recon[2::3])] if ctype==2 else \
                      [1 if px<threshold else 0 for px in recon])
            matrix.append(row_px)
        return matrix, w, h


# =========================================================================
# BƯỚC 2: DÒ TÌM ĐƯỜNG BIÊN NGOÀI & LỖ HỔNG (MOORE-NEIGHBOR TRACING)
# =========================================================================
class BoundaryDetector:
    """Stage 2: Thuật toán dò biên Moore-Neighbor 8 hướng theo chiều kim đồng hồ."""
    @classmethod
    def find_all_contours(cls, matrix, w, h):
        visited, contours = set(), []
        dirs = [(-1,0), (-1,-1), (0,-1), (1,-1), (1,0), (1,1), (0,1), (-1,1)]
        for y in range(h):
            if 1 not in matrix[y]: continue
            for x in range(w):
                if matrix[y][x] != 1 or (x, y) in visited: continue
                if not any(nx<0 or nx>=w or ny<0 or ny>=h or matrix[ny][nx]==0 for dx,dy in dirs for nx,ny in [(x+dx,y+dy)]): continue
                contour, cx, cy, d_idx = [], x, y, 0
                while True:
                    contour.append((cx, cy)); visited.add((cx, cy))
                    found = False
                    for i in range(8):
                        tidx = (d_idx + i) % 8
                        nx, ny = cx + dirs[tidx][0], cy + dirs[tidx][1]
                        if 0 <= nx < w and 0 <= ny < h and matrix[ny][nx] == 1:
                            cx, cy, d_idx, found = nx, ny, (tidx + 5) % 8, True; break
                    if not found or (cx == x and cy == y): break
                if len(contour) >= 10: contours.append(contour)
        return contours


# =========================================================================
# BƯỚC 3: PHÓNG TIA DDA TÌM TRUNG ĐIỂM (UNIFIED EXPERIMENTAL RAY-CASTER)
# =========================================================================
class RayCasterDDA:
    """Stage 3: Phóng tia vuông góc tìm trung điểm nét vẽ, phân nhánh theo EXPERIMENTAL_RUN."""
    _apex_anchors = []

    @staticmethod
    def get_normal(matrix, w, h, contour, idx):
        # Cửa sổ pháp tuyến động: Khúc cua nhọn dùng k=2 để không cắt chéo, đoạn thẳng dùng k=5
        n = len(contour)
        p_prev, p_mid, p_next = contour[(idx-5)%n], contour[idx], contour[(idx+5)%n]
        t1x, t1y = p_mid[0]-p_prev[0], p_mid[1]-p_prev[1]
        t2x, t2y = p_next[0]-p_mid[0], p_next[1]-p_mid[1]
        l1, l2 = math.hypot(t1x, t1y), math.hypot(t2x, t2y)
        k = 2 if (l1>1e-6 and l2>1e-6 and (t1x*t2x + t1y*t2y)/(l1*l2) < 0.5) else 5
        
        p0, pe = contour[(idx-k)%n], contour[(idx+k)%n]
        dx, dy = pe[0]-p0[0], pe[1]-p0[1]
        length = math.hypot(dx, dy)
        if length < 1e-6: return (0.0, 0.0)
        n1, n2 = (-dy/length, dx/length), (dy/length, -dx/length)
        px, py = contour[idx]
        def probe(nx, ny, eps):
            ix, iy = int(round(px + eps*nx)), int(round(py + eps*ny))
            return 0 <= ix < w and 0 <= iy < h and matrix[iy][ix] == 1
        for eps in (2.0, 1.0, 0.5):
            if probe(n1[0], n1[1], eps) and not probe(n2[0], n2[1], eps): return n1
            if probe(n2[0], n2[1], eps) and not probe(n1[0], n1[1], eps): return n2
        return (0.0, 0.0)

    @classmethod
    def cast_rays(cls, matrix, contours, w, h):
        cls._apex_anchors = []
        b_norms = {p: n for contour in contours for i, p in enumerate(contour) if (n := cls.get_normal(matrix, w, h, contour, i)) != (0.0, 0.0)}
        
        # Nhận diện các điểm chóp nhọn nhất trên biên (< 50°) lưu vào _apex_anchors
        for contour in contours:
            n = len(contour)
            for i in range(n):
                p0, p1, p2 = contour[(i-4)%n], contour[i], contour[(i+4)%n]
                v1x, v1y = p0[0]-p1[0], p0[1]-p1[1]
                v2x, v2y = p2[0]-p1[0], p2[1]-p1[1]
                l1, l2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
                if l1 > 1e-6 and l2 > 1e-6 and (v1x*v2x + v1y*v2y)/(l1*l2) > 0.64: # góc < ~50°
                    cls._apex_anchors.append(p1)

        midpoints = []
        for contour in contours:
            for px, py in contour:
                nx, ny = b_norms.get((px, py), (0.0, 0.0))
                if nx == 0 and ny == 0: continue

                # ── [TẬP 2]: Lọc bỏ tia DDA xuất phát trong bán kính 18px quanh chóp để chống nhiễu ──
                if EXPERIMENTAL_RUN == "Tập 2" and any(math.hypot(px-ax, py-ay) < 18.0 for ax, ay in cls._apex_anchors):
                    continue

                # ── [TẬP 1]: Xoay nghiêng tia DDA hướng về đỉnh chóp khi gần góc nhọn ──
                if EXPERIMENTAL_RUN == "Tập 1" and cls._apex_anchors:
                    ax, ay = min(cls._apex_anchors, key=lambda a: math.hypot(px-a[0], py-a[1]))
                    if (d_apex := math.hypot(px-ax, py-ay)) < 25.0 and d_apex > 1e-6:
                        wx, wy = (ax-px)/d_apex, (ay-py)/d_apex
                        if nx*wx + ny*wy > 0:
                            warp_f = (1.0 - d_apex/25.0) * 0.55
                            nx, ny = nx*(1-warp_f) + wx*warp_f, ny*(1-warp_f) + wy*warp_f
                            norm = math.hypot(nx, ny)
                            if norm > 1e-6: nx, ny = nx/norm, ny/norm

                t, hit, qx, qy = 0.5, False, float(px), float(py)
                while t <= 350.0:
                    ix, iy = int(round(px + t*nx)), int(round(py + t*ny))
                    if not (0 <= ix < w and 0 <= iy < h): break
                    if matrix[iy][ix] == 0: hit, qx, qy = True, float(ix), float(iy); break
                    t += 0.5
                if not hit or t < 1.0: continue

                mx, my = (px + qx)*0.5, (py + qy)*0.5
                qnx, qny = (0.0, 0.0)
                for r in range(1, 4):
                    for dx, dy in [(-r,0),(r,0),(0,-r),(0,r)]:
                        if (qn := b_norms.get((int(qx+dx), int(qy+dy)), (0.0, 0.0))) != (0.0, 0.0):
                            qnx, qny = qn; break
                    if qnx != 0 or qny != 0: break
                
                dot = nx*qnx + ny*qny if (qnx!=0 or qny!=0) else 0.0
                if (qnx==0 and qny==0) or (t < 8.0 and dot < -0.15) or (t >= 8.0 and dot < -0.3):
                    midpoints.append((mx, my, t, nx, ny))
        return midpoints


# =========================================================================
# BƯỚC 4: XỬ LÝ ĐỒ THỊ, NỐI PHAO & CẮT NGÃ BA (UNIFIED TOPOLOGY RESOLVER)
# =========================================================================
class TopologyResolver:
    """Stage 4: Phân vùng lưới cell, co cụm ngã ba, co giãn RNG và cắt râu rác cụt."""
    @staticmethod
    def _remove_node(nodes, nid):
        if nid in nodes:
            for nbr in list(nodes[nid][5]):
                if nbr in nodes: nodes[nbr][5].discard(nid)
            del nodes[nid]

    @staticmethod
    def build_graph_and_segment(midpoints, return_edges=False):
        if not midpoints: return ([], []) if return_edges else []
        avg_w = sum(m[2] for m in midpoints) / len(midpoints)
        cell_size = max(6.0, min(14.0, avg_w * 0.18))
        bucket_sums = defaultdict(lambda: [0.0, 0.0, 0.0, 0, 0.0, 0.0])
        thin_clusters = []

        # ── PHÂN VÙNG & CO CỤM TRUNG ĐIỂM (SPATIAL HASHING / ANGLE CLUSTERING) ──
        for mx, my, w, nx, ny in midpoints:
            # [TẬP 1]: Gom nhóm điểm mỏng theo hướng vectơ thay vì chia lưới vuông
            if EXPERIMENTAL_RUN == "Tập 1" and w < 0.35 * avg_w:
                matched = False
                for tc in thin_clusters:
                    if math.hypot(mx - tc[0]/tc[3], my - tc[1]/tc[3]) < 1.5 * cell_size:
                        nn = math.hypot(tc[4], tc[5])
                        if abs(nx * (tc[4]/nn if nn>1e-6 else 0) + ny * (tc[5]/nn if nn>1e-6 else 0)) > 0.7 or tc[3] == 0:
                            tc[0]+=mx; tc[1]+=my; tc[2]+=w; tc[3]+=1; tc[4]+=nx; tc[5]+=ny; matched = True; break
                if not matched: thin_clusters.append([mx, my, w, 1, nx, ny])
            else:
                b = bucket_sums[(int(mx // cell_size), int(my // cell_size))]
                b[0]+=mx; b[1]+=my; b[2]+=w; b[3]+=1; b[4]+=nx; b[5]+=ny

        nodes = {}
        for i, d in enumerate(bucket_sums.values()):
            nn = math.hypot(d[4], d[5])
            nodes[i] = [d[0]/d[3], d[1]/d[3], d[2]/d[3], (d[4]/nn if nn>1e-6 else 0.0), (d[5]/nn if nn>1e-6 else 0.0), set()]
        next_id = max(nodes.keys()) + 1 if nodes else 0
        for tc in thin_clusters:
            nn = math.hypot(tc[4], tc[5])
            nodes[next_id] = [tc[0]/tc[3], tc[1]/tc[3], tc[2]/tc[3], (tc[4]/nn if nn>1e-6 else 0.0), (tc[5]/nn if nn>1e-6 else 0.0), set()]
            next_id += 1

        grid = defaultdict(list)
        for i, nd in nodes.items(): grid[(int(nd[0]//cell_size), int(nd[1]//cell_size))].append(i)

        # ── XÂY DỰNG ĐỒ THỊ RNG (RELATIVE NEIGHBORHOOD GRAPH) CÓ KẾT HỢP GÓC PHÁP TUYẾN ──
        for u_id, u in nodes.items():
            gx, gy = int(u[0]//cell_size), int(u[1]//cell_size)
            search_r = int(math.ceil(1.8 * max(u[2], 80.0) * 1.5 / cell_size)) + 1
            local_ids = [v_id for dx in range(-search_r, search_r+1) for dy in range(-search_r, search_r+1)
                         for v_id in grid.get((gx+dx, gy+dy), []) if v_id != u_id]
            for v_id in local_ids:
                if u_id < v_id:
                    v = nodes[v_id]
                    dist = math.hypot(u[0]-v[0], u[1]-v[1])
                    max_r = (1.2 + 0.6 * abs(u[3]*v[3] + u[4]*v[4])) * max(u[2], v[2])
                    if 1e-4 < dist < max_r and all(max(math.hypot(u[0]-nodes[z][0], u[1]-nodes[z][1]), math.hypot(v[0]-nodes[z][0], v[1]-nodes[z][1])) >= dist for z in local_ids if z != v_id):
                        u[5].add(v_id); v[5].add(u_id)

        # ── [TẬP 2]: Sub-Pixel Corner Snap -> Kéo thẳng trục trung vị bisector nối từ thân tới chóp ──
        if EXPERIMENTAL_RUN == "Tập 2" and RayCasterDDA._apex_anchors:
            thin_ids = [i for i, nd in nodes.items() if nd[2] < 0.35 * avg_w]
            visited_thin = set()
            for t_id in thin_ids:
                if t_id in visited_thin or t_id not in nodes: continue
                cluster, q = [], [t_id]
                visited_thin.add(t_id)
                while q:
                    curr = q.pop(0); cluster.append(curr)
                    for nbr in nodes[curr][5]:
                        if nbr in thin_ids and nbr not in visited_thin: visited_thin.add(nbr); q.append(nbr)
                if len(cluster) >= 2:
                    cluster.sort(key=lambda i: nodes[i][2], reverse=True)
                    rx, ry = nodes[cluster[0]][0], nodes[cluster[0]][1]
                    best_ax, best_ay = min(RayCasterDDA._apex_anchors, key=lambda a: math.hypot(rx-a[0], ry-a[1]))
                    dx, dy = best_ax - rx, best_ay - ry
                    length = math.hypot(dx, dy)
                    if length > 1e-6:
                        ux, uy = dx/length, dy/length
                        for c_id in cluster[1:]:
                            nd = nodes[c_id]
                            t_proj = max(0.0, min(length, (nd[0]-rx)*ux + (nd[1]-ry)*uy))
                            nd[0], nd[1] = rx + t_proj*ux, ry + t_proj*uy
                        for c_id in cluster:
                            for nbr in list(nodes[c_id][5]):
                                if nbr in cluster: nodes[c_id][5].discard(nbr)
                        for k in range(len(cluster)-1):
                            nodes[cluster[k]][5].add(cluster[k+1]); nodes[cluster[k+1]][5].add(cluster[k])

        # ── HEURISTIC 1: Co cụm ngã ba gần nhau (< 0.8 * width) có bảo vệ vùng chóp ──
        apex_protected = {i for i, nd in nodes.items() if nd[2] < 0.25 * avg_w}
        changed = True
        while changed:
            changed = False
            j_ids = [i for i, nd in nodes.items() if len(nd[5]) >= 3]
            for i in range(len(j_ids)):
                u_id = j_ids[i]
                if u_id not in nodes or u_id in apex_protected: continue
                u = nodes[u_id]
                for j in range(i+1, len(j_ids)):
                    v_id = j_ids[j]
                    if v_id not in nodes or v_id in apex_protected: continue
                    v = nodes[v_id]
                    if math.hypot(u[0]-v[0], u[1]-v[1]) < 0.8 * min(u[2], v[2]):
                        u[0]=(u[0]+v[0])*0.5; u[1]=(u[1]+v[1])*0.5; u[2]=(u[2]+v[2])*0.5
                        u[5].discard(v_id); v[5].discard(u_id)
                        for nbr in list(v[5]):
                            if nbr in nodes and nbr != u_id: nodes[nbr][5].discard(v_id); nodes[nbr][5].add(u_id); u[5].add(nbr)
                        u[5].discard(u_id); del nodes[v_id]; changed = True

        # ── HEURISTIC 2: Cắt tỉa nhánh cụt ngắn (< 1.0 * width) không làm cụt chóp mũi tên ──
        changed = True
        while changed:
            changed = False
            for ep_id in [i for i, nd in list(nodes.items()) if len(nd[5]) == 1]:
                if ep_id not in nodes or ep_id in apex_protected: continue
                path, curr, arc_len = [ep_id], ep_id, 0.0
                while True:
                    nbrs = [n for n in nodes[curr][5] if len(path) < 2 or n != path[-2]]
                    if not nbrs: break
                    nxt = nbrs[0]
                    arc_len += math.hypot(nodes[curr][0]-nodes[nxt][0], nodes[curr][1]-nodes[nxt][1])
                    path.append(nxt); curr = nxt
                    if len(nodes[curr][5]) != 2: break
                if len(nodes[curr][5]) >= 3 and arc_len < 1.0 * nodes[curr][2]:
                    if nodes[ep_id][2] < 0.25 * avg_w: continue
                    for p_id in path[:-1]: TopologyResolver._remove_node(nodes, p_id)
                    changed = True
                elif len(nodes[curr][5]) <= 1 and arc_len < 2.0 * avg_w and len(path) >= 2:
                    if nodes[path[0]][2] < 0.25 * avg_w or nodes[curr][2] < 0.25 * avg_w: continue
                    if (sn:=nodes.get(path[0])) and (en:=nodes.get(curr)) and arc_len > 0 and math.hypot(sn[0]-en[0], sn[1]-en[1])/arc_len < 0.45:
                        for p_id in path: TopologyResolver._remove_node(nodes, p_id)
                        changed = True

        # ── BÓC TÁCH NHÁNH VẼ TỪ ĐỒ THỊ ──
        visited_edges, branches = set(), []
        start_ids = [i for i, nd in nodes.items() if len(nd[5]) != 2] or list(nodes.keys())
        for s_id in start_ids:
            if s_id not in nodes: continue
            for nbr_id in list(nodes[s_id][5]):
                edge = tuple(sorted((s_id, nbr_id)))
                if edge in visited_edges: continue
                pts, curr, prev = [(nodes[s_id][0], nodes[s_id][1], nodes[s_id][2])], nbr_id, s_id
                visited_edges.add(edge)
                while True:
                    if curr not in nodes: break
                    pts.append((nodes[curr][0], nodes[curr][1], nodes[curr][2]))
                    if len(nodes[curr][5]) != 2 and curr != s_id: break
                    next_ids = [n for n in nodes[curr][5] if n != prev]
                    if not next_ids: break
                    nxt = next_ids[0]; next_edge = tuple(sorted((curr, nxt)))
                    if next_edge in visited_edges:
                        if nxt in nodes: pts.append((nodes[nxt][0], nodes[nxt][1], nodes[nxt][2]))
                        break
                    visited_edges.add(next_edge); prev, curr = curr, nxt
                if len(pts) >= 2:
                    branch_len = sum(math.hypot(pts[j+1][0]-pts[j][0], pts[j+1][1]-pts[j][1]) for j in range(len(pts)-1))
                    chord_len = math.hypot(pts[0][0]-pts[-1][0], pts[0][1]-pts[-1][1])
                    is_sharp_tip = (pts[0][2] < 0.35 * avg_w or pts[-1][2] < 0.35 * avg_w)
                    if branch_len >= 0.9 * avg_w or is_sharp_tip:
                        if chord_len < 1.0 and branch_len < 3.0 * avg_w: continue
                        if branch_len > 0 and chord_len / branch_len < 0.2 and branch_len < 3.0 * avg_w: continue
                        branches.append(pts)
        edges = [(u[0], u[1], nodes[v_id][0], nodes[v_id][1]) for u_id, u in nodes.items() for v_id in u[5] if u_id < v_id and v_id in nodes] if return_edges else []
        return (branches, edges) if return_edges else branches


# =========================================================================
# BƯỚC 5: LÀM MƯỢT VÀ XUẤT SVG (UNIFIED EXPERIMENTAL CURVE SMOOTHER)
# =========================================================================
class CurveSmoother:
    """Stage 5: Đơn giản hóa RDP và chuyển thành đường cong Bézier có hỗ trợ nêm nhọn."""
    @classmethod
    def rdp(cls, pts, eps=None):
        if len(pts) < 3: return pts
        p0, pe = pts[0], pts[-1]
        dx, dy = pe[0]-p0[0], pe[1]-p0[1]
        dist_sq = dx*dx + dy*dy
        max_d, max_i = max((abs(dy*p[0] - dx*p[1] + pe[0]*p0[1] - pe[1]*p0[0]) / math.sqrt(dist_sq) if dist_sq > 0 else math.hypot(p[0]-p0[0], p[1]-p0[1]), i) for i, p in enumerate(pts[1:-1], 1))
        
        # ── [TẬP 3]: RDP có sai số động eps theo độ cong tiếp tuyến cục bộ ──
        if EXPERIMENTAL_RUN == "Tập 3":
            t1x, t1y = pts[max_i][0]-pts[max_i-1][0], pts[max_i][1]-pts[max_i-1][1]
            t2x, t2y = pts[max_i+1][0]-pts[max_i][0], pts[max_i+1][1]-pts[max_i][1]
            l1, l2 = math.hypot(t1x, t1y), math.hypot(t2x, t2y)
            curv = max(0.0, 1.0 - (t1x*t2x + t1y*t2y)/(l1*l2)) if l1>1e-6 and l2>1e-6 else 0.0
            eps_eff = 0.5 if curv > 0.4 else (3.5 if curv < 0.05 else (eps or 1.5))
        else:
            if eps is None:
                chord_len = math.hypot(dx, dy)
                arc_len = sum(math.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1]) for i in range(len(pts)-1))
                eps_eff = 3.0 if (arc_len > 0 and chord_len / arc_len > 0.98) else 1.2
            else:
                eps_eff = eps

        return cls.rdp(pts[:max_i+1], eps_eff)[:-1] + cls.rdp(pts[max_i:], eps_eff) if max_d > eps_eff else [p0, pe]

    @staticmethod
    def to_bezier_svg(pts, avg_w=None):
        if not pts: return ""
        if len(pts) == 1: return f"M {pts[0][0]:.2f} {pts[0][1]:.2f}"
        if len(pts) == 2: return f"M {pts[0][0]:.2f} {pts[0][1]:.2f} L {pts[1][0]:.2f} {pts[1][1]:.2f}"
        
        if avg_w is None:
            w_vals = [p[2] for p in pts if len(p) >= 3]
            avg_w = sum(w_vals)/len(w_vals) if w_vals else 100.0

        # ── [TẬP 1]: Cắt bỏ 15% điểm nhiễu cuối nhánh chóp & hồi quy thẳng tới apex anchor ──
        if EXPERIMENTAL_RUN == "Tập 1" and len(pts) >= 4 and pts[-1][2] < 0.35 * avg_w:
            n_cut = max(1, int(len(pts) * 0.15))
            stable_pts = pts[:-n_cut] if len(pts)-n_cut >= 2 else pts[:2]
            best_ax, best_ay = min(RayCasterDDA._apex_anchors, key=lambda a: math.hypot(pts[-1][0]-a[0], pts[-1][1]-a[1])) if RayCasterDDA._apex_anchors else pts[-1][:2]
            if math.hypot(pts[-1][0]-best_ax, pts[-1][1]-best_ay) < 3.0 * avg_w:
                pts = stable_pts + [(best_ax, best_ay, 0.0)]

        # ── [TẬP 3]: Ngoại suy đỉnh chóp thêm 0.5 * width theo vectơ tiếp tuyến cuối ──
        if EXPERIMENTAL_RUN == "Tập 3" and len(pts) >= 3 and pts[-1][2] < 0.35 * avg_w:
            dx, dy = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
            norm = math.hypot(dx, dy)
            if norm > 1e-6:
                ext_d = pts[-1][2] * 0.5
                pts = pts[:-1] + [(pts[-1][0] + dx/norm*ext_d, pts[-1][1] + dy/norm*ext_d, 0.0)]

        n, tans = len(pts), []
        for i in range(n):
            tx, ty = (pts[1][0]-pts[0][0], pts[1][1]-pts[0][1]) if i==0 else ((pts[-1][0]-pts[-2][0], pts[-1][1]-pts[-2][1]) if i==n-1 else (pts[i+1][0]-pts[i-1][0], pts[i+1][1]-pts[i-1][1]))
            norm = math.hypot(tx, ty)
            tans.append((tx/norm, ty/norm) if norm > 1e-6 else (0.0, 0.0))

        cmds = [f"M {pts[0][0]:.2f} {pts[0][1]:.2f}"]
        w_start, w_end = (pts[0][2] if len(pts[0])>=3 else avg_w), (pts[-1][2] if len(pts[-1])>=3 else avg_w)
        tip_at_start, tip_at_end = (w_start < 0.35 * avg_w), (w_end < 0.35 * avg_w)

        for i in range(n - 1):
            p1, p2, t1, t2 = pts[i], pts[i+1], tans[i], tans[i+1]
            dist, dot_t = math.hypot(p2[0]-p1[0], p2[1]-p1[1]), t1[0]*t2[0] + t1[1]*t2[1]

            # ── [TẬP 2]: Chuyển tiếp Quadratic Bézier (Q) rồi kết thúc bằng đoạn thẳng (L) ──
            if EXPERIMENTAL_RUN == "Tập 2":
                if i == n - 2 and tip_at_end: cmds.append(f"L {p2[0]:.2f} {p2[1]:.2f}"); continue
                elif i == n - 3 and tip_at_end: cmds.append(f"Q {p1[0]+dist*0.25*t1[0]:.2f} {p1[1]+dist*0.25*t1[1]:.2f}, {p2[0]:.2f} {p2[1]:.2f}"); continue
                elif i == 0 and tip_at_start: cmds.append(f"L {p2[0]:.2f} {p2[1]:.2f}"); continue
                elif i == 1 and tip_at_start: cmds.append(f"Q {p2[0]-dist*0.25*t2[0]:.2f} {p2[1]-dist*0.25*t2[1]:.2f}, {p2[0]:.2f} {p2[1]:.2f}"); continue

            # ── [TẬP 3 & chung]: Clamping tay quay về 0 cho đỉnh chóp để ép tuyến tính L ──
            if (i == 0 and tip_at_start) or (i == n - 2 and tip_at_end):
                if EXPERIMENTAL_RUN == "Tập 3": cmds.append(f"L {p2[0]:.2f} {p2[1]:.2f}"); continue
                else: d = dist * 0.05
            else:
                d = dist * 0.33 * (0.5 + 0.5 * dot_t)

            cmds.append(f"C {p1[0]+d*t1[0]:.2f} {p1[1]+d*t1[1]:.2f}, {p2[0]-d*t2[0]:.2f} {p2[1]-d*t2[1]:.2f}, {p2[0]:.2f} {p2[1]:.2f}")
        return " ".join(cmds)


# =========================================================================
# BƯỚC 6: ĐIỀU HÀNH PIPELINE & XUẤT SVG/DEBUG DƯỚI 15 DÒNG XML
# =========================================================================
def process_image(png_path, svg_path):
    print(f"\n  [>>> THÍ NGHIỆM ĐANG CHẠY: {EXPERIMENTAL_RUN} <<<]")
    matrix, w, h = PurePNG.read_binary_matrix(png_path)
    print(f"  [Bước 1] Đọc ảnh thô: {w}x{h}, {sum(sum(row) for row in matrix)} điểm đen (pixels)")

    contours = BoundaryDetector.find_all_contours(matrix, w, h)
    print(f"  [Bước 2] Dò biên: tìm thấy {len(contours)} đường viền ngoài & lỗ hổng")

    midpoints = RayCasterDDA.cast_rays(matrix, contours, w, h)
    print(f"  [Bước 3] Phóng tia DDA: xác định {len(midpoints)} điểm tim sông (trung điểm)")

    avg_w = sum(m[2] for m in midpoints) / len(midpoints) if midpoints else 100.0
    branches, edges = TopologyResolver.build_graph_and_segment(midpoints, return_edges=True)
    print(f"  [Bước 4] Xử lý đồ thị: phân đoạn thành {len(branches)} nét vẽ, {len(edges)} cạnh RNG")

    svg = ET.Element('svg', {'xmlns': 'http://www.w3.org/2000/svg', 'width': '1024', 'height': '1024', 'viewBox': '0 0 1024 1024', 'preserveAspectRatio': 'none'})
    g = ET.SubElement(svg, 'g', {'id': 'skeleton-shapes'})
    for idx, branch in enumerate(branches, 1):
        if path_d := CurveSmoother.to_bezier_svg(CurveSmoother.rdp(branch), avg_w=avg_w):
            ET.SubElement(g, 'path', {'id': f'path-{idx}-1', 'd': path_d, 'fill': 'none', 'stroke': 'black', 'stroke-width': '45', 'stroke-linecap': 'round', 'stroke-linejoin': 'round'})
    ET.indent(ET.ElementTree(svg), space="  "); ET.ElementTree(svg).write(svg_path, encoding='utf-8', xml_declaration=True)

    debug_svg_path = os.path.splitext(svg_path)[0] + "_debug.svg"
    svg_dbg = ET.Element('svg', {'xmlns': 'http://www.w3.org/2000/svg', 'width': '1024', 'height': '1024', 'viewBox': '0 0 1024 1024', 'preserveAspectRatio': 'none'})
    g_edges = ET.SubElement(svg_dbg, 'g', {'id': 'rng-edges'})
    for x1, y1, x2, y2 in edges: ET.SubElement(g_edges, 'line', {'x1': f'{x1:.2f}', 'y1': f'{y1:.2f}', 'x2': f'{x2:.2f}', 'y2': f'{y2:.2f}', 'stroke': 'red', 'stroke-width': '1.5'})
    g_mids = ET.SubElement(svg_dbg, 'g', {'id': 'raw-midpoints'})
    for mx, my, _, _, _ in midpoints: ET.SubElement(g_mids, 'circle', {'cx': f'{mx:.2f}', 'cy': f'{my:.2f}', 'r': '1.8', 'fill': 'green'})
    ET.indent(ET.ElementTree(svg_dbg), space="  "); ET.ElementTree(svg_dbg).write(debug_svg_path, encoding='utf-8', xml_declaration=True)

    print(f"  [Bước 5] Làm mượt & xuất SVG: đã lưu '{svg_path}' và '{debug_svg_path}'")
    return len(branches)


if __name__ == "__main__":
    if len(sys.argv) > 2:
        curves = process_image(sys.argv[1], sys.argv[2])
        print(f"Hoàn thành trích xuất '{sys.argv[1]}' -> '{sys.argv[2]}' ({curves} nét vẽ)")
    else:
        sample_dir, output_dir = "challenge_sample", "challenge_sample_output"
        if os.path.exists(sample_dir):
            os.makedirs(output_dir, exist_ok=True)
            png_files = sorted(glob.glob(os.path.join(sample_dir, "*.png")))
            print(f"Tìm thấy {len(png_files)} ảnh mẫu trong '{sample_dir}'. Đang trích xuất xương...")
            for i, png_file in enumerate(png_files, 1):
                base_name = os.path.splitext(os.path.basename(png_file))[0]
                curves = process_image(png_file, os.path.join(output_dir, f"{base_name}.svg"))
                print(f"[{i}/{len(png_files)}] {base_name}.png -> {base_name}.svg ({curves} nét vẽ)")
            print(f"\nHoàn tất 100%! Tất cả file SVG đã lưu vào '{output_dir}'.")
        else:
            print("Cú pháp: python centerline_extractor.py <input.png> <output.svg>")