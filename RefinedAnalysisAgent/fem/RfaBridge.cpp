///////////////////////////////////////////////////////////////////////
// RfaBridge - refined live load analysis of a girder bridge (see RfaBridge.h)
///////////////////////////////////////////////////////////////////////
#include "RfaBridge.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>

namespace rfa
{
   namespace
   {
      P2 Lerp(const P2& a, const P2& b, double t) { return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t }; }
      double Dist(const P2& a, const P2& b) { return std::hypot(b.x - a.x, b.y - a.y); }
      P2 Sub(const P2& a, const P2& b) { return { a.x - b.x, a.y - b.y }; }
      double Dot(const P2& a, const P2& b) { return a.x * b.x + a.y * b.y; }

      std::string Fmt(double v, int prec = 1)
      {
         std::ostringstream os; os.setf(std::ios::fixed); os.precision(prec); os << v; return os.str();
      }

      enum RowKind { RowPier1, RowEnd1, RowFrac, RowEnd2, RowPier2 };
      struct RowDef { RowKind kind; double f; };

      // girder fraction of an arbitrary point along brg1->brg2
      double FracOf(const GirderLine& g, const P2& p)
      {
         P2 d = Sub(g.brg2, g.brg1);
         return Dot(Sub(p, g.brg1), d) / Dot(d, d);
      }

      P2 RowPoint(const GirderLine& g, const RowDef& r)
      {
         switch (r.kind)
         {
         case RowPier1: return g.pier1;
         case RowEnd1: return g.end1;
         case RowEnd2: return g.end2;
         case RowPier2: return g.pier2;
         default: return Lerp(g.brg1, g.brg2, r.f);
         }
      }

      // Transverse partition counts: [left overhang, bays..., right overhang]
      std::vector<int> Partition(const std::vector<double>& widths, int total, const MeshOptions& mo, size_t nBays)
      {
         size_t np = widths.size();
         std::vector<int> n(np);
         for (size_t k = 0; k < np; k++)
         {
            bool bay = (k > 0 && k + 1 < np);
            if (bay) n[k] = std::max(mo.minPerBay, (int)std::ceil(widths[k] / mo.transverseSize - 1e-9));
            else n[k] = widths[k] < 0.02 ? 0 : std::max(1, (int)std::ceil(widths[k] / mo.transverseSize - 1e-9));
         }
         (void)nBays;
         if (total > 0)
         {
            int sum = 0; for (int v : n) sum += v;
            while (sum < total)
            {
               // add one division where the element is currently widest
               size_t best = 0; double bw = -1;
               for (size_t k = 0; k < np; k++)
               {
                  double w = widths[k] / std::max(n[k], 1);
                  if (widths[k] < 0.02) continue;
                  if (w > bw) { bw = w; best = k; }
               }
               n[best]++; sum++;
            }
         }
         return n;
      }

      double RectJ(double a, double b)
      {
         // St. Venant torsion constant of an a x b rectangle (a >= b)
         if (a < b) std::swap(a, b);
         if (b <= 0) return 0;
         double r = b / a;
         return a * b * b * b * (1.0 / 3.0 - 0.21 * r * (1 - r * r * r * r / 12.0));
      }
   }

   ////////////////////////////////////////////////////////////////////
   Results RunRefinedAnalysis(const BridgeInput& in, const AnalysisOptions& opt)
   {
      auto t0 = std::chrono::steady_clock::now();
      Results res;
      auto progress = [&](const std::string& s) { if (opt.progress) opt.progress(s); };
      const MeshOptions& mo = opt.mesh;
      const size_t nSpans = in.spans.size();
      if (nSpans == 0 || in.piers.size() != nSpans + 1) { res.error = "bridge description is incomplete (spans/piers)"; return res; }
      for (size_t i = 0; i < nSpans; i++)
         if (in.spans[i].girders.size() < 2) { res.error = "span " + std::to_string(i + 1) + " has fewer than two girders; a plate model needs at least two"; return res; }

      // effective support kinds (continuity needs the same girder count on both sides)
      std::vector<PierInput> piers = in.piers;
      piers.front().kind = SupportKind::Abutment;
      piers.back().kind = SupportKind::Abutment;
      for (size_t p = 1; p < nSpans; p++)
      {
         if (piers[p].kind == SupportKind::ContinuousPier && in.spans[p - 1].girders.size() != in.spans[p].girders.size())
         {
            res.notes.push_back("Pier " + std::to_string(p + 1) + ": girder count changes across the pier, modelled as simple supports (no girder continuity)");
            piers[p].kind = SupportKind::SimplePier;
         }
      }
      auto deckShared = [&](size_t p) { return piers[p].kind == SupportKind::ContinuousPier || (piers[p].kind == SupportKind::SimplePier && piers[p].deckContinuous); };

      progress("Meshing");
      // ---------------- rows per span ----------------
      struct SpanMesh
      {
         std::vector<RowDef> rows;
         std::vector<std::vector<P2>> pts;     // [row][transverse node]
         std::vector<std::vector<int>> nodes;  // node ids
         std::vector<int> part;                // partition counts
         std::vector<int> kg;                  // girder position in row
         int rowBrg1 = 0, rowBrg2 = 0, rowFirstGirder = 0, rowLastGirder = 0;
         std::map<long long, int> fracRow;     // rounded f*1e6 -> row
         double Lbrg = 0;
         std::vector<double> fShear;           // shear section fractions (start, end)
      };
      std::vector<SpanMesh> sm(nSpans);

      for (size_t i = 0; i < nSpans; i++)
      {
         const SpanInput& sp = in.spans[i];
         SpanMesh& m = sm[i];
         double L = 0; for (auto& g : sp.girders) L += Dist(g.brg1, g.brg2);
         L /= sp.girders.size();
         m.Lbrg = L;
         int N = 10 * std::max(1, (int)std::ceil(L / (10 * mo.longitudinalSize) - 1e-9));
         std::vector<double> fr;
         for (int k = 0; k <= N; k++) fr.push_back((double)k / N);
         // shear sections at (girder + deck) height from the bearings
         double H = 0; for (auto& g : sp.girders) H = std::max(H, g.section(0.5).height);
         H += sp.deckT;
         double fv = std::clamp(H / L, 1.0 / N, 0.2);
         m.fShear = { fv, 1 - fv };
         std::vector<double> extra = { fv, 1 - fv };
         for (auto& d : sp.diaphragms) extra.push_back(d.f);
         for (double f : extra)
         {
            if (f <= 0 || f >= 1) continue;
            bool near = false;
            for (double e : fr) if (std::fabs(e - f) < 0.25 / N) { near = true; break; }
            if (!near) fr.push_back(f);
         }
         std::sort(fr.begin(), fr.end());

         const GirderLine& g0 = sp.girders.front();
         const double tol = 0.05;
         if (piers[i].kind != SupportKind::Abutment) m.rows.push_back({ RowPier1, 0 });
         if (Dist(g0.end1, g0.brg1) > tol && (piers[i].kind == SupportKind::Abutment || Dist(g0.end1, g0.pier1) > tol)) m.rows.push_back({ RowEnd1, 0 });
         m.rowBrg1 = (int)m.rows.size();
         for (double f : fr) m.rows.push_back({ RowFrac, f });
         m.rowBrg2 = (int)m.rows.size() - 1;
         if (Dist(g0.end2, g0.brg2) > tol && (piers[i + 1].kind == SupportKind::Abutment || Dist(g0.end2, g0.pier2) > tol)) m.rows.push_back({ RowEnd2, 0 });
         if (piers[i + 1].kind != SupportKind::Abutment) m.rows.push_back({ RowPier2, 0 });
         m.rowFirstGirder = (m.rows.front().kind == RowPier1) ? 1 : 0;
         m.rowLastGirder = (m.rows.back().kind == RowPier2) ? (int)m.rows.size() - 2 : (int)m.rows.size() - 1;
         for (int r = 0; r < (int)m.rows.size(); r++) if (m.rows[r].kind == RowFrac) m.fracRow[std::llround(m.rows[r].f * 1e6)] = r;
      }

      // ---------------- transverse partition (same total everywhere) ----------------
      auto edgePoint = [&](const P2& gEx, const P2& gIn, double edgeOffset) -> P2
      {
         double s0, o0, s1, o1;
         in.alignment.inverse(gEx, s0, o0);
         in.alignment.inverse(gIn, s1, o1);
         double len = Dist(gEx, gIn);
         P2 d = { (gEx.x - gIn.x) / len, (gEx.y - gIn.y) / len };
         double rate = (o0 - o1) / len; // offset change per unit length moving outward
         if (std::fabs(rate) < 1e-9) return gEx;
         double t = (edgeOffset - o0) / rate;
         if (t < 0) t = 0;
         return { gEx.x + d.x * t, gEx.y + d.y * t };
      };
      auto rowWidths = [&](size_t i, const RowDef& r, std::vector<P2>& gp, P2& eL, P2& eR)
      {
         const SpanInput& sp = in.spans[i];
         gp.clear();
         for (auto& g : sp.girders) gp.push_back(RowPoint(g, r));
         double s, o;
         in.alignment.inverse(gp.front(), s, o);
         eL = edgePoint(gp[0], gp[1], in.leftDeckEdge(s));
         in.alignment.inverse(gp.back(), s, o);
         eR = edgePoint(gp.back(), gp[gp.size() - 2], in.rightDeckEdge(s));
         std::vector<double> w;
         w.push_back(Dist(eL, gp[0]));
         for (size_t k = 0; k + 1 < gp.size(); k++) w.push_back(Dist(gp[k], gp[k + 1]));
         w.push_back(Dist(gp.back(), eR));
         return w;
      };
      int T = 0;
      std::vector<std::vector<double>> spanW(nSpans);
      for (size_t i = 0; i < nSpans; i++)
      {
         // widest of start / middle / end rows
         std::vector<double> wmax;
         for (auto& r : sm[i].rows)
         {
            std::vector<P2> gp; P2 a, b;
            auto w = rowWidths(i, r, gp, a, b);
            if (wmax.empty()) wmax = w; else for (size_t k = 0; k < w.size(); k++) wmax[k] = std::max(wmax[k], w[k]);
         }
         spanW[i] = wmax;
         auto n = Partition(wmax, 0, mo, wmax.size() - 2);
         int sum = 0; for (int v : n) sum += v;
         T = std::max(T, sum);
      }
      // spans joined by girder continuity must put each girder on the same transverse node
      for (size_t i = 1; i < nSpans; i++)
         if (piers[i].kind == SupportKind::ContinuousPier)
            for (size_t k = 0; k < spanW[i].size(); k++) spanW[i][k] = std::max(spanW[i][k], spanW[i - 1][k]);
      for (size_t i = nSpans - 1; i-- > 0;)
         if (piers[i + 1].kind == SupportKind::ContinuousPier) spanW[i] = spanW[i + 1];
      for (size_t i = 0; i < nSpans; i++)
      {
         sm[i].part = Partition(spanW[i], T, mo, spanW[i].size() - 2);
         sm[i].kg.clear();
         int k = sm[i].part[0];
         for (size_t g = 0; g < in.spans[i].girders.size(); g++) { sm[i].kg.push_back(k); if (g + 1 < sm[i].part.size()) k += sm[i].part[g + 1]; }
      }

      // ---------------- node positions ----------------
      for (size_t i = 0; i < nSpans; i++)
      {
         SpanMesh& m = sm[i];
         for (auto& r : m.rows)
         {
            std::vector<P2> gp; P2 eL, eR;
            rowWidths(i, r, gp, eL, eR);
            std::vector<P2> key; key.push_back(eL); for (auto& p : gp) key.push_back(p); key.push_back(eR);
            std::vector<P2> pts;
            for (size_t part = 0; part < m.part.size(); part++)
               for (int k = 0; k < m.part[part]; k++) pts.push_back(Lerp(key[part], key[part + 1], (double)k / m.part[part]));
            pts.push_back(m.part.back() == 0 ? gp.back() : eR);
            m.pts.push_back(pts);
         }
      }

      // ---------------- create nodes (sharing deck rows across piers) ----------------
      Model model;
      for (size_t i = 0; i < nSpans; i++)
      {
         SpanMesh& m = sm[i];
         m.nodes.assign(m.rows.size(), std::vector<int>(T + 1, -1));
         for (size_t r = 0; r < m.rows.size(); r++)
         {
            bool share = (r == 0 && i > 0 && deckShared(i));
            for (int k = 0; k <= T; k++)
            {
               if (share)
               {
                  const SpanMesh& pm = sm[i - 1];
                  m.nodes[r][k] = pm.nodes.back()[k];
               }
               else
               {
                  P2 p = m.pts[r][k];
                  // shared row: average with the previous span's last row
                  if (r + 1 == m.rows.size() && i + 1 < nSpans && deckShared(i + 1))
                  {
                     const P2& q = sm[i + 1].pts.front()[k];
                     p = Lerp(p, q, 0.5);
                  }
                  m.nodes[r][k] = model.AddNode(p.x, p.y);
               }
            }
         }
      }

      // ---------------- shells ----------------
      std::vector<std::vector<std::vector<int>>> shellAt(nSpans); // [span][row strip r..r+1][k]
      for (size_t i = 0; i < nSpans; i++)
      {
         const SpanInput& sp = in.spans[i];
         ShellProps p; p.E = sp.deckE; p.nu = sp.deckNu; p.t = sp.deckT;
         int pid = model.AddShellProps(p);
         SpanMesh& m = sm[i];
         shellAt[i].assign(m.rows.size(), std::vector<int>(T, -1));
         for (size_t r = 0; r + 1 < m.rows.size(); r++)
            for (int k = 0; k < T; k++)
            {
               int a = m.nodes[r][k], b = m.nodes[r][k + 1], c = m.nodes[r + 1][k + 1], d = m.nodes[r + 1][k];
               const Vec3& A = model.NodePos(a); const Vec3& B = model.NodePos(b); const Vec3& C = model.NodePos(c); const Vec3& D = model.NodePos(d);
               double area = 0.5 * std::fabs((C - A).cross(D - B).z);
               if (area < 1e-6) continue;
               shellAt[i][r][k] = model.AddShell(a, b, c, d, pid);
            }
      }

      // ---------------- girders ----------------
      // girderElems[span][girder][row] = beam element ending at that row (left), -1 if none
      std::vector<std::vector<std::vector<int>>> elemLeft(nSpans), elemRight(nSpans);
      std::vector<std::vector<std::vector<GirderSection>>> secAt(nSpans);
      for (size_t i = 0; i < nSpans; i++)
      {
         const SpanInput& sp = in.spans[i];
         SpanMesh& m = sm[i];
         size_t G = sp.girders.size();
         elemLeft[i].assign(G, std::vector<int>(m.rows.size(), -1));
         elemRight[i].assign(G, std::vector<int>(m.rows.size(), -1));
         secAt[i].assign(G, std::vector<GirderSection>(m.rows.size()));
         for (size_t g = 0; g < G; g++)
         {
            const GirderLine& gl = sp.girders[g];
            for (size_t r = 0; r < m.rows.size(); r++)
            {
               double f = m.rows[r].kind == RowFrac ? m.rows[r].f : FracOf(gl, RowPoint(gl, m.rows[r]));
               f = std::clamp(f, (double)FracOf(gl, gl.end1), (double)FracOf(gl, gl.end2));
               secAt[i][g][r] = gl.section(f);
            }
            int r0 = m.rowFirstGirder, r1 = m.rowLastGirder;
            // continuity across a continuous pier: girder runs to the pier node
            if (m.rows.front().kind == RowPier1 && piers[i].kind == SupportKind::ContinuousPier) r0 = 0;
            if (m.rows.back().kind == RowPier2 && piers[i + 1].kind == SupportKind::ContinuousPier) r1 = (int)m.rows.size() - 1;
            for (int r = r0; r < r1; r++)
            {
               const GirderSection& sa = secAt[i][g][r];
               const GirderSection& sb = secAt[i][g][r + 1];
               BeamProps bp;
               bp.E = 0.5 * (sa.E + sb.E); bp.G = bp.E / (2 * 1.2);
               bp.A = 0.5 * (sa.A + sb.A); bp.Iv = 0.5 * (sa.Iv + sb.Iv); bp.Il = 0.5 * (sa.Il + sb.Il); bp.J = 0.5 * (sa.J + sb.J);
               int pid = model.AddBeamProps(bp);
               int e = model.AddBeam(m.nodes[r][m.kg[g]], m.nodes[r + 1][m.kg[g]], pid, sa.ecc, sb.ecc);
               elemRight[i][g][r] = e;
               elemLeft[i][g][r + 1] = e;
            }
         }
      }

      // ---------------- diaphragms ----------------
      auto addDiaphragmRow = [&](size_t i, int r, int gFrom, int gTo, double E, double W, double H, double ecc)
      {
         if (W <= 0 || H <= 0 || E <= 0) return;
         BeamProps bp; bp.E = E; bp.G = E / 2.4; bp.A = W * H; bp.Iv = W * H * H * H / 12; bp.Il = H * W * W * W / 12; bp.J = RectJ(W, H);
         int pid = model.AddBeamProps(bp);
         const SpanMesh& m = sm[i];
         for (int k = m.kg[gFrom]; k < m.kg[gTo]; k++) model.AddBeam(m.nodes[r][k], m.nodes[r][k + 1], pid, ecc, ecc);
      };
      for (size_t i = 0; i < nSpans; i++)
      {
         const SpanInput& sp = in.spans[i];
         SpanMesh& m = sm[i];
         int G = (int)sp.girders.size();
         for (auto& d : sp.diaphragms)
         {
            // nearest row
            int best = m.rowBrg1; double bd = 1e9;
            for (int r = m.rowBrg1; r <= m.rowBrg2; r++) if (std::fabs(m.rows[r].f - d.f) < bd) { bd = std::fabs(m.rows[r].f - d.f); best = r; }
            addDiaphragmRow(i, best, std::clamp(d.gFrom, 0, G - 1), std::clamp(d.gTo, 0, G - 1), d.E, d.width, d.height, d.ecc);
         }
         for (int end = 0; end < 2; end++)
         {
            double W = sp.endDiaphragmW[end], H = sp.endDiaphragmH[end];
            size_t p = i + end;
            int r;
            if (piers[p].kind == SupportKind::ContinuousPier) { if (end == 0) continue; r = (int)m.rows.size() - 1; } // once, on the pier row
            else r = end == 0 ? m.rowBrg1 : m.rowBrg2;
            addDiaphragmRow(i, r, 0, G - 1, sp.endDiaphragmE, W, H, sp.deckT / 2 + H / 2);
         }
      }

      // ---------------- supports ----------------
      struct Support { size_t span; int girder; int row; int node; };
      std::vector<Support> supports;
      for (size_t i = 0; i < nSpans; i++)
      {
         const SpanInput& sp = in.spans[i];
         SpanMesh& m = sm[i];
         for (int end = 0; end < 2; end++)
         {
            size_t p = i + end;
            int r;
            if (piers[p].kind == SupportKind::ContinuousPier)
            {
               if (end == 0) continue; // added by the previous span on the shared pier row
               r = (int)m.rows.size() - 1;
            }
            else r = end == 0 ? m.rowBrg1 : m.rowBrg2;
            for (size_t g = 0; g < sp.girders.size(); g++)
            {
               const GirderLine& gl = sp.girders[g];
               int nd = m.nodes[r][m.kg[g]];
               const GirderSection& s = secAt[i][g][r];
               model.FixDof(nd, W);
               P2 t = Sub(gl.brg2, gl.brg1); double tl = std::hypot(t.x, t.y); t = { t.x / tl, t.y / tl };
               Vec3 ndir(-t.y, t.x, 0);
               (void)gl;
               bool integral = piers[p].integral[end == 0 ? 1 : 0];
               if (piers[p].kind == SupportKind::ContinuousPier) integral = piers[p].integral[0] || piers[p].integral[1];
               if (integral) model.AddRotationalSpring(nd, ndir, 1e3 * 4 * s.E * s.Iv / std::max(m.Lbrg, 1.0));
               supports.push_back({ i, (int)g, r, nd });
            }
         }
      }
      // Horizontal restraint: statically determinate per structural unit (spans joined by a shared
      // deck), like the pin/roller of the line girder: longitudinal + transverse at the middle girder
      // of the unit's first support line, transverse at the middle girder of its last support line.
      // Vertical loads then produce no horizontal reactions (no spurious arching or girder-to-girder
      // self-equilibrating bearing forces).
      {
         size_t i = 0;
         while (i < nSpans)
         {
            size_t j = i;
            while (j + 1 < nSpans && deckShared(j + 1)) j++;
            const Support* first = nullptr; const Support* last = nullptr;
            int gmA = (int)in.spans[i].girders.size() / 2, gmB = (int)in.spans[j].girders.size() / 2;
            for (auto& s : supports)
            {
               if (s.span == i && s.girder == gmA && (s.row == sm[i].rowBrg1 || s.row == 0) && !first) first = &s;
               if (s.span == j && s.girder == gmB) last = &s; // the last one added for span j is its end line
            }
            if (first && last && first != last)
            {
               const GirderLine& ga = in.spans[i].girders[gmA];
               P2 t = Sub(ga.brg2, ga.brg1); double tl = std::hypot(t.x, t.y); t = { t.x / tl, t.y / tl };
               double k = mo.bearingSpringFactor * secAt[i][gmA][first->row].E * secAt[i][gmA][first->row].A / std::max(sm[i].Lbrg, 1.0);
               model.AddSpring(first->node, Vec3(t.x, t.y, 0), Vec3(0, 0, 0), k);
               model.AddSpring(first->node, Vec3(-t.y, t.x, 0), Vec3(0, 0, 0), k);
               const GirderLine& gb = in.spans[j].girders[gmB];
               P2 u = Sub(gb.brg2, gb.brg1); double ul = std::hypot(u.x, u.y); u = { u.x / ul, u.y / ul };
               model.AddSpring(last->node, Vec3(-u.y, u.x, 0), Vec3(0, 0, 0), k);
            }
            i = j + 1;
         }
      }
      res.nodes = (int)model.NumNodes(); res.shells = (int)model.NumShells(); res.beams = (int)model.NumBeams();

      progress("Factoring stiffness (" + std::to_string(model.NumNodes() * 6) + " dof)");
      std::string err;
      if (!model.Factor(&err)) { res.error = "FE model: " + err; return res; }
      res.equations = model.NumEquations();

      // ---------------- equilibrium self-check ----------------
      auto selfCheckPoints = [&](size_t i) -> std::vector<P2>
      {
         // a few deck points in span i: girder lines and mid-bays at 0.37L / 0.5L / 0.81L
         std::vector<P2> pts;
         const SpanInput& sp = in.spans[i];
         double fs[3] = { 0.37, 0.5, 0.81 };
         for (double f : fs)
         {
            P2 a = Lerp(sp.girders[0].brg1, sp.girders[0].brg2, f);
            P2 b = Lerp(sp.girders[1].brg1, sp.girders[1].brg2, f);
            P2 c2 = Lerp(sp.girders.back().brg1, sp.girders.back().brg2, f);
            pts.push_back(a); pts.push_back(Lerp(a, b, 0.5)); pts.push_back(c2);
         }
         return pts;
      };
      double worstReaction = 0;

      // ---------------- response functionals ----------------
      struct Resp
      {
         size_t span; int girder; int row; double f; std::string label; Action action; int sign; bool twoTrucks;
         Functional fn;
         // line-girder counterpart
         int lineNode = -1; int lineElem = -1; int lineEnd = 1;
      };
      std::vector<Resp> resp;

      auto girderDir = [&](size_t i, int g) { const GirderLine& gl = in.spans[i].girders[g]; P2 t = Sub(gl.brg2, gl.brg1); double l = std::hypot(t.x, t.y); return P2{ t.x / l, t.y / l }; };

      // Section resultant of girder g's tributary strip on the cut along row r, from the nodal
      // forces of the elements on one side of the cut (exact equilibrium, any load position).
      // leftSide: elements between rows r-1 and r (a load on the cut counts as right of it);
      // otherwise elements between r and r+1 (a load on the cut counts as left of it).
      // shear = false: sagging moment about the horizontal axis normal to the girder through
      // the girder point; shear = true: vertical shear (positive = up on the left part).
      auto sectionFn = [&](size_t i, int g, int r, bool leftSide, bool shear) -> Functional
      {
         Functional F;
         const SpanMesh& m = sm[i];
         int G = (int)in.spans[i].girders.size();
         int rr = leftSide ? r - 1 : r;
         if (rr < 0 || rr + 1 >= (int)m.rows.size()) { leftSide = !leftSide; rr = leftSide ? r - 1 : r; }
         if (rr < 0 || rr + 1 >= (int)m.rows.size()) return F;
         P2 t = girderDir(i, g);
         Vec3 yh(-t.y, t.x, 0);
         const Vec3& P = model.NodePos(m.nodes[r][m.kg[g]]);
         double kl = g > 0 ? 0.5 * (m.kg[g - 1] + m.kg[g]) : -1e9;
         double kr = g + 1 < G ? 0.5 * (m.kg[g] + m.kg[g + 1]) : 1e9;
         auto weight = [&](int k) { if (k > kl && k < kr) return 1.0; if (k == kl || k == kr) return 0.5; return 0.0; };
         double sgn = leftSide ? -1.0 : 1.0;
         auto addNode = [&](const Functional* rows /*6*/, int nd, double w)
         {
            const Vec3& Q = model.NodePos(nd);
            Vec3 rv = Q - P;
            for (int d = 0; d < 6; d++)
            {
               double c;
               if (shear) c = (d == W) ? 1.0 : 0.0;
               else if (d < 3) { Vec3 e(d == 0, d == 1, d == 2); c = rv.cross(e).dot(yh); }
               else { Vec3 e(d == 3, d == 4, d == 5); c = e.dot(yh); }
               // moment: sagging = -My(positive face); shear: V = -Vz(positive face)
               if (c != 0) F.add(rows[d], sgn * w * c);
            }
         };
         for (int k = 0; k < T && mo.includePlateInGirderActions; k++)
         {
            int e = shellAt[i][rr][k];
            if (e < 0) continue;
            const Shell& s = model.GetShell(e);
            for (int ln = 0; ln < 4; ln++)
            {
               int nd = s.n[ln];
               int kk = -1;
               for (int q = k; q <= k + 1; q++) if (m.nodes[r][q] == nd) kk = q;
               if (kk < 0) continue;
               double w = weight(kk);
               if (w == 0) continue;
               Functional rows[6];
               for (int d = 0; d < 6; d++) rows[d] = model.ShellNodalForce(e, ln, d);
               addNode(rows, nd, w);
            }
         }
         int be = leftSide ? elemLeft[i][g][r] : elemRight[i][g][r];
         if (be >= 0)
         {
            int ln = leftSide ? 1 : 0;
            Functional rows[6];
            for (int d = 0; d < 6; d++) rows[d] = model.BeamNodalForce(be, ln, d);
            addNode(rows, model.GetBeam(be).n[ln], 1.0);
         }
         return F;
      };
      auto momentFn = [&](size_t i, int g, int r) { return sectionFn(i, g, r, true, false); };
      auto shearFn = [&](size_t i, int g, int r, bool startSide) { return sectionFn(i, g, r, startSide, true); };

      for (size_t i = 0; i < nSpans; i++)
      {
         SpanMesh& m = sm[i];
         int G = (int)in.spans[i].girders.size();
         bool contStart = piers[i].kind == SupportKind::ContinuousPier, contEnd = piers[i + 1].kind == SupportKind::ContinuousPier;
         bool anyCont = contStart || contEnd || piers[i].integral[1] || piers[i + 1].integral[0];
         for (int g = 0; g < G; g++)
         {
            for (int k = 1; k <= 9; k++)
            {
               double f = k / 10.0;
               int r = m.fracRow.at(std::llround(f * 1e6));
               Functional F = momentFn(i, g, r);
               resp.push_back({ i, g, r, f, Fmt(f) + "L", Action::MomentPos, +1, false, F });
               if (anyCont) resp.push_back({ i, g, r, f, Fmt(f) + "L", Action::MomentNeg, -1, true, F });
            }
            // shear sections
            for (int side = 0; side < 2; side++)
            {
               double f = m.fShear[side];
               int r = m.rowBrg1; double bd = 1e9;
               for (int q = m.rowBrg1; q <= m.rowBrg2; q++) if (std::fabs(m.rows[q].f - f) < bd) { bd = std::fabs(m.rows[q].f - f); r = q; }
               Functional F = shearFn(i, g, r, side == 0);
               resp.push_back({ i, g, r, m.rows[r].f, side == 0 ? "shear (start)" : "shear (end)", Action::Shear, side == 0 ? +1 : -1, false, F });
            }
            // negative moment over a continuous pier (end of span, once per pier)
            if (contEnd)
            {
               int r = (int)m.rows.size() - 1;
               Functional F = momentFn(i, g, r);
               resp.push_back({ i, g, r, FracOf(in.spans[i].girders[g], in.spans[i].girders[g].pier2), "Pier " + std::to_string(i + 2), Action::MomentNeg, -1, true, F });
            }
         }
      }
      for (auto& s : supports)
      {
         Functional F = model.Reaction(s.node, W);
         bool interior = (s.row == (int)sm[s.span].rows.size() - 1 && piers[s.span + 1].kind == SupportKind::ContinuousPier);
         const SpanMesh& m = sm[s.span];
         double f = (s.row == m.rowBrg1) ? 0.0 : (s.row == m.rowBrg2 ? 1.0 : FracOf(in.spans[s.span].girders[s.girder], in.spans[s.span].girders[s.girder].pier2));
         resp.push_back({ s.span, s.girder, s.row, f, interior ? "reaction (pier)" : (f < 0.5 ? "reaction (start)" : "reaction (end)"), Action::Reaction, +1, interior, F });
      }

      // reactions sum to P everywhere; girder moments at 0.5L sum to the static moment (simple straight spans)
      for (size_t i = 0; i < nSpans; i++)
      {
         std::vector<Functional> m05;
         std::vector<int> m05g;
         for (auto& R : resp) if (R.span == i && R.action == Action::MomentPos && std::fabs(R.f - 0.5) < 1e-9) { m05.push_back(R.fn); m05g.push_back(R.girder); }
         bool simple = piers[i].kind != SupportKind::ContinuousPier && piers[i + 1].kind != SupportKind::ContinuousPier && !piers[i].integral[1] && !piers[i + 1].integral[0];
         // straight & square: every girder parallel and bearings on a line normal to them
         const SpanInput& sp = in.spans[i];
         P2 t0 = Sub(sp.girders[0].brg2, sp.girders[0].brg1); double L0 = std::hypot(t0.x, t0.y); t0 = { t0.x / L0, t0.y / L0 };
         bool square = true;
         for (auto& g : sp.girders)
         {
            P2 t = Sub(g.brg2, g.brg1); double L = std::hypot(t.x, t.y);
            if (std::fabs(L - L0) > 1e-3 * L0 || std::fabs((t.x * t0.y - t.y * t0.x) / L) > 1e-4) square = false;
            if (std::fabs(Dot(Sub(g.brg1, sp.girders[0].brg1), t0)) > 1e-3 * L0) square = false;
         }
         double worstM = 0;
         for (const P2& p : selfCheckPoints(i))
         {
            std::vector<double> u = model.SolvePointLoad(p.x, p.y, 1.0);
            double R = 0;
            for (auto& s : supports) R += model.Evaluate(model.Reaction(s.node, W), u);
            worstReaction = std::max(worstReaction, std::fabs(R - 1.0));
            if (simple && square)
            {
               double a = Dot(Sub(p, sp.girders[0].brg1), t0);
               double Mstatic = a <= 0.5 * L0 ? a * 0.5 : 0.5 * (L0 - a);
               double Msum = 0; for (auto& f : m05) Msum += model.Evaluate(f, u);
               worstM = std::max(worstM, std::fabs(Msum - Mstatic) / (0.25 * L0));
            }
         }
         if (simple && square)
            res.notes.push_back("Self-check span " + std::to_string(i + 1) + ": sum of girder moments at 0.5L vs statics, worst error " + Fmt(100 * worstM, 3) + "% of PL/4");
      }
      res.notes.push_back("Self-check: sum of bearing reactions vs applied load, worst error " + Fmt(100 * worstReaction, 4) + "%");

      progress("Influence surfaces (" + std::to_string(resp.size()) + ")");
      std::vector<Functional> fns; for (auto& r : resp) fns.push_back(r.fn);
      auto IS = model.InfluenceSurfaces(fns);

      // ---------------- roadway grid ----------------
      double sMin = 1e300, sMax = -1e300;
      for (size_t i = 0; i < nSpans; i++)
         for (auto& row : sm[i].pts) for (const P2& p : { row.front(), row.back() })
         {
            double s, o; in.alignment.inverse(p, s, o);
            sMin = std::min(sMin, s); sMax = std::max(sMax, s);
         }
      double roadL = -1e300, roadR = 1e300;
      for (int k = 0; k <= 50; k++)
      {
         double s = sMin + (sMax - sMin) * k / 50.0;
         roadL = std::max(roadL, in.leftCurb(s));
         roadR = std::min(roadR, in.rightCurb(s));
      }
      res.roadLeft = roadL; res.roadRight = roadR;
      res.designLanes = (int)std::floor((roadR - roadL) / opt.live.laneWidth + 1e-9);
      if (res.designLanes < 1) { res.error = "roadway narrower than one design lane"; return res; }

      Grid2D grid;
      grid.ds = mo.gridStep; grid.dO = mo.gridStep;
      grid.s0 = sMin; grid.ns = (int)std::ceil((sMax - sMin) / grid.ds) + 1;
      grid.o0 = roadL; grid.no = (int)std::floor((roadR - roadL) / grid.dO + 1e-9) + 1;
      std::vector<PointLoc> locs((size_t)grid.ns * grid.no);
      for (int k = 0; k < grid.ns; k++)
      {
         P2 pt, nrm;
         in.alignment.eval(grid.s0 + k * grid.ds, pt, nrm);
         for (int j = 0; j < grid.no; j++)
         {
            double o = grid.o0 + j * grid.dO;
            locs[(size_t)k * grid.no + j] = model.Locate(pt.x + nrm.x * o, pt.y + nrm.y * o);
         }
      }

      {
         size_t miss = 0;
         for (auto& l : locs) if (l.shell < 0) miss++;
         res.notes.push_back("Roadway grid " + std::to_string(grid.ns) + " x " + std::to_string(grid.no) + ", points off the deck: " + std::to_string(miss));
      }

      // ---------------- line girder models (one per girder line chain) ----------------
      // nodes along the developed girder length; supports and continuity as in the plate model
      struct LineModel
      {
         std::unique_ptr<Model> model;
         std::map<std::pair<size_t, int>, int> nodeOf; // (span,row) -> node
         std::map<std::pair<size_t, int>, int> eLeft, eRight;
         std::vector<double> u;  // node coordinate
      };
      std::map<std::pair<size_t, int>, int> chainOf; // (span, girder) -> chain id
      std::vector<LineModel> chains;
      {
         // chain = maximal run of spans joined by continuous piers with the same girder index
         size_t i = 0;
         while (i < nSpans)
         {
            size_t j = i;
            while (j + 1 < nSpans && piers[j + 1].kind == SupportKind::ContinuousPier) j++;
            int G = (int)in.spans[i].girders.size();
            for (int g = 0; g < G; g++)
            {
               LineModel lm; lm.model = std::make_unique<Model>();
               Model& M = *lm.model;
               double u = 0; int prevNode = -1; P2 prevPt{};
               std::vector<int> supNodes;
               for (size_t q = i; q <= j; q++)
               {
                  SpanMesh& m = sm[q];
                  const GirderLine& gl = in.spans[q].girders[g];
                  int r0 = (q == i) ? m.rowFirstGirder : 0;
                  int r1 = (q == j) ? m.rowLastGirder : (int)m.rows.size() - 1;
                  for (int r = r0; r <= r1; r++)
                  {
                     if (q > i && r == 0) { lm.nodeOf[{q, r}] = prevNode; continue; } // shared pier node
                     P2 p = RowPoint(gl, m.rows[r]);
                     if (prevNode >= 0) u += Dist(prevPt, p);
                     int nd = M.AddNode(u, 0);
                     lm.u.push_back(u);
                     lm.nodeOf[{q, r}] = nd;
                     if (prevNode >= 0)
                     {
                        // composite stiffness of the strip
                        const GirderSection& sa = secAt[q][g][std::max(r - 1, 0)];
                        const GirderSection& sb = secAt[q][g][r];
                        auto Ic = [&](const GirderSection& s)
                        {
                           double w = 0.5 * Dist(in.spans[q].girders[std::max(g - 1, 0)].brg1, in.spans[q].girders[std::min(g + 1, (int)in.spans[q].girders.size() - 1)].brg1);
                           double nrat = in.spans[q].deckE / std::max(s.E, 1.0);
                           double Ad = nrat * w * in.spans[q].deckT, Id = nrat * w * std::pow(in.spans[q].deckT, 3) / 12;
                           double yb = s.A * s.ecc / (s.A + Ad);
                           return s.Iv + s.A * (s.ecc - yb) * (s.ecc - yb) + Id + Ad * yb * yb;
                        };
                        BeamProps bp; bp.E = 0.5 * (sa.E + sb.E); bp.G = bp.E / 2.4; bp.A = 0.5 * (sa.A + sb.A);
                        bp.Iv = 0.5 * (Ic(sa) + Ic(sb)); bp.Il = bp.Iv; bp.J = bp.Iv;
                        int pid = M.AddBeamProps(bp);
                        int e = M.AddBeam(prevNode, nd, pid, 0, 0);
                        // map elements to (span,row)
                        int rPrev = r - 1;
                        size_t qPrev = q;
                        if (r == 0 && q > i) { qPrev = q - 1; rPrev = (int)sm[q - 1].rows.size() - 1; }
                        lm.eRight[{qPrev, rPrev}] = e;
                        lm.eLeft[{q, r}] = e;
                        // the shared pier node is also (q, 0) of the next span
                     }
                     prevNode = nd; prevPt = p;
                  }
                  // the next span's row 0 (pier) element mapping
                  if (q < j) { lm.eLeft[{q + 1, 0}] = lm.eLeft[{q, r1}]; }
               }
               // supports from the plate model
               for (auto& s : supports)
               {
                  if (s.girder != g || s.span < i || s.span > j) continue;
                  int nd = lm.nodeOf.at({ s.span, s.row });
                  M.FixDof(nd, W);
                  bool integral = false;
                  if (s.row == sm[s.span].rowBrg1) integral = piers[s.span].integral[1];
                  if (s.row == sm[s.span].rowBrg2) integral = piers[s.span + 1].integral[0];
                  if (s.row == (int)sm[s.span].rows.size() - 1 && piers[s.span + 1].kind == SupportKind::ContinuousPier)
                     integral = piers[s.span + 1].integral[0] || piers[s.span + 1].integral[1];
                  if (integral) M.FixDof(nd, RY);
               }
               for (size_t n = 0; n < M.NumNodes(); n++) { M.FixDof((int)n, U); M.FixDof((int)n, V); M.FixDof((int)n, RX); M.FixDof((int)n, RZ); }
               std::string e2;
               if (!M.Factor(&e2)) { res.error = "line girder model: " + e2; return res; }
               for (size_t q = i; q <= j; q++) chainOf[{q, g}] = (int)chains.size();
               chains.push_back(std::move(lm));
            }
            i = j + 1;
         }
      }

      // ---------------- search ----------------
      progress("Placing HL-93 on " + std::to_string(resp.size()) + " surfaces");
      SearchOptions so; so.roadLeft = roadL; so.roadRight = roadR;
      std::map<std::pair<size_t, int>, std::vector<size_t>> byGirder;
      for (size_t q = 0; q < resp.size(); q++)
      {
         Resp& R = resp[q];
         grid.v.assign((size_t)grid.ns * grid.no, 0.0);
         for (size_t p = 0; p < locs.size(); p++) grid.v[p] = model.Interpolate(IS[q], locs[p]);
         so.sign = R.sign; so.twoTrucks = R.twoTrucks;
         LiveResult lr = SearchSurface(grid, opt.live, so);

         // line girder
         LineModel& lm = chains[chainOf.at({ R.span, R.girder })];
         Model& M = *lm.model;
         Functional LF;
         int nd = lm.nodeOf.at({ R.span, R.row });
         if (R.action == Action::Reaction) LF = M.Reaction(nd, W);
         else
         {
            bool startSide = (R.action == Action::Shear && R.sign > 0);
            auto itL = lm.eLeft.find({ R.span, R.row }), itR = lm.eRight.find({ R.span, R.row });
            int e = -1, end = 1;
            if (R.action == Action::Shear)
            {
               if (startSide) { if (itL != lm.eLeft.end()) { e = itL->second; end = 1; } else if (itR != lm.eRight.end()) { e = itR->second; end = 0; } }
               else { if (itR != lm.eRight.end()) { e = itR->second; end = 0; } else if (itL != lm.eLeft.end()) { e = itL->second; end = 1; } }
               if (e >= 0) LF = M.BeamShear(e, end);
            }
            else
            {
               if (itL != lm.eLeft.end()) { e = itL->second; end = 1; } else if (itR != lm.eRight.end()) { e = itR->second; end = 0; }
               if (e >= 0) LF = M.BeamMomentSag(e, end);
            }
         }
         auto IL = M.InfluenceSurface(LF);
         // resample on a uniform grid along u
         double u0 = lm.u.front(), u1 = lm.u.back();
         int nu = (int)std::ceil((u1 - u0) / grid.ds) + 1;
         std::vector<double> ILg(nu);
         size_t seg = 0;
         for (int k = 0; k < nu; k++)
         {
            double u = std::min(u0 + k * grid.ds, u1);
            while (seg + 2 < lm.u.size() && lm.u[seg + 1] < u) seg++;
            double ua = lm.u[seg], ub = lm.u[seg + 1];
            double t = ub > ua ? (u - ua) / (ub - ua) : 0;
            ILg[k] = IL[seg] + (IL[seg + 1] - IL[seg]) * t;
         }
         LiveResult ll = SearchLine(ILg, u0, grid.ds, opt.live, R.sign, R.twoTrucks);

         StationResult st;
         st.span = (int)R.span; st.girder = R.girder; st.f = R.f; st.label = R.label; st.action = R.action;
         st.fe = lr.value; st.line = ll.value;
         st.g = std::fabs(ll.value) > 0 ? lr.value / ll.value : 0;
         st.feOneLaneNoMpf = lr.byLanes.empty() ? 0 : lr.byLanes[0] / opt.live.MPF(1);
         st.gFatigue = std::fabs(ll.value) > 0 ? st.feOneLaneNoMpf / ll.value : 0;
         st.governing = lr;
         bool keep = opt.keepSurfaces && ((R.action == Action::MomentPos && std::fabs(R.f - 0.5) < 1e-6) || R.label == "shear (start)" || (R.action == Action::MomentNeg && R.label.rfind("Pier", 0) == 0));
         if (keep)
         {
            st.surfaceId = (int)res.surfaces.size();
            res.surfaces.push_back({ "Span " + std::to_string(R.span + 1) + " Girder " + std::string(1, (char)('A' + R.girder)) + " " + R.label + (R.action == Action::MomentNeg ? " -M" : R.action == Action::MomentPos ? " +M" : ""), grid });
         }
         byGirder[{R.span, R.girder}].push_back(res.stations.size());
         res.stations.push_back(st);
         if (q % 20 == 0) progress("Placing HL-93: " + std::to_string(q + 1) + " / " + std::to_string(resp.size()));
      }

      // ---------------- distribution factors ----------------
      // Distribution factor = (FE envelope maximum) / (line-girder envelope maximum) over the
      // stations of one action (AASHTO 4.6.3 practice: compare the extreme effects). With
      // opt.stationRatio the largest station-by-station ratio is used instead (stations where the
      // line-girder effect is at least half its maximum).
      auto ratioOver = [&](const std::vector<size_t>& idx, Action a, bool fatigue, bool pierOnly) -> double
      {
         double maxLine = 0, maxFE = 0, gs = 0;
         for (size_t k : idx)
         {
            auto& s = res.stations[k];
            if (s.action != a || (pierOnly && s.label.rfind("Pier", 0) != 0)) continue;
            maxLine = std::max(maxLine, std::fabs(s.line));
            maxFE = std::max(maxFE, std::fabs(fatigue ? s.feOneLaneNoMpf : s.fe));
         }
         if (maxLine <= 0) return 0;
         if (!opt.stationRatio) return maxFE / maxLine;
         for (size_t k : idx)
         {
            auto& s = res.stations[k];
            if (s.action != a || (pierOnly && s.label.rfind("Pier", 0) != 0)) continue;
            if (std::fabs(s.line) < 0.5 * maxLine) continue;
            gs = std::max(gs, fatigue ? s.gFatigue : s.g);
         }
         return gs;
      };
      for (size_t i = 0; i < nSpans; i++)
         for (int g = 0; g < (int)in.spans[i].girders.size(); g++)
         {
            auto& idx = byGirder[{i, g}];
            GirderLLDF d; d.span = (int)i; d.girder = g;
            d.gMpos = ratioOver(idx, Action::MomentPos, false, false);
            d.gMposF = ratioOver(idx, Action::MomentPos, true, false);
            d.gV = 0; d.gVF = 0;
            for (size_t k : idx)
            {
               auto& s = res.stations[k];
               if (s.action != Action::Shear) continue;
               d.gV = std::max(d.gV, s.g); d.gVF = std::max(d.gVF, s.gFatigue);
            }
            double gn = ratioOver(idx, Action::MomentNeg, false, false), gnF = ratioOver(idx, Action::MomentNeg, true, false);
            // include the adjacent pier stations (stored with the span ending at the pier)
            if (i > 0) { auto& pidx = byGirder[{i - 1, g}]; gn = std::max(gn, ratioOver(pidx, Action::MomentNeg, false, true)); gnF = std::max(gnF, ratioOver(pidx, Action::MomentNeg, true, true)); }
            d.gMneg = gn > 0 ? gn : d.gMpos;
            d.gMnegF = gnF > 0 ? gnF : d.gMposF;
            res.girders.push_back(d);
         }
      for (size_t p = 0; p <= nSpans; p++)
      {
         size_t G = (p < nSpans ? in.spans[p] : in.spans[p - 1]).girders.size();
         for (int g = 0; g < (int)G; g++)
         {
            PierLLDF d; d.pier = (int)p; d.girder = g;
            if (p > 0 && p < nSpans && piers[p].kind == SupportKind::ContinuousPier)
            {
               auto& idx = byGirder[{p - 1, g}];
               d.gMneg = ratioOver(idx, Action::MomentNeg, false, true);
               d.gMnegF = ratioOver(idx, Action::MomentNeg, true, true);
            }
            else
            {
               // not used by PGSuper (no continuity), carry the adjacent span's value
               size_t s = std::min(p, nSpans - 1);
               for (auto& gd : res.girders) if (gd.span == (int)s && gd.girder == g) { d.gMneg = gd.gMneg; d.gMnegF = gd.gMnegF; }
            }
            res.piers.push_back(d);
         }
      }

      for (size_t i = 0; i < nSpans; i++)
         for (int g = 0; g < (int)in.spans[i].girders.size(); g++)
         {
            Results::GirderSO gs; gs.span = (int)i; gs.girder = g;
            in.alignment.inverse(in.spans[i].girders[g].end1, gs.s1, gs.o1);
            in.alignment.inverse(in.spans[i].girders[g].end2, gs.s2, gs.o2);
            res.girderLines.push_back(gs);
         }
      // mesh for plots
      for (size_t n = 0; n < model.NumNodes(); n++) res.meshNodes.push_back({ model.NodePos((int)n).x, model.NodePos((int)n).y });
      for (size_t e = 0; e < model.NumShells(); e++) { auto& s = model.GetShell((int)e); res.meshShells.push_back(s.n); }
      for (size_t e = 0; e < model.NumBeams(); e++) { auto& b = model.GetBeam((int)e); res.meshBeams.push_back(b.n); }

      res.ok = true;
      res.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      return res;
   }
}
