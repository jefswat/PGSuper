// Synthetic bridges through the full refined analysis pipeline
#include "../fem/RfaBridge.h"
#include <cstdio>
#include <cmath>
#include <string>

using namespace rfa;
static const double FT = 0.3048, IN = 0.0254, KSI = 6.894757e6;

struct Spec
{
   int nG = 5; double S = 8, overhang = 3, L = 100, skewDeg = 0; int spans = 1; bool continuous = false;
   double curbIn = 1.5; // curb inset from deck edge (ft)
};

static BridgeInput MakeBridge(const Spec& sp)
{
   BridgeInput b;
   // Type IV girder
   const double A = 789 * IN * IN, I = 260741 * IN * IN * IN * IN, Iy = 24374 * IN * IN * IN * IN, yb = 24.73 * IN, h = 54 * IN;
   const double Eg = 5000 * KSI, Ed = 3600 * KSI, ts = 7.5 * IN, haunch = 1.0 * IN;
   const double Ip = I + Iy, J = A * A * A * A / (40 * Ip);
   const double ecc = (h - yb) + haunch + ts / 2;
   double skew = sp.skewDeg * 3.14159265358979 / 180;
   double width = (sp.nG - 1) * sp.S + 2 * sp.overhang;
   for (int i = 0; i < sp.spans; i++)
   {
      SpanInput s;
      s.deckE = Ed; s.deckNu = 0.2; s.deckT = ts;
      double x0 = i * sp.L * FT;
      for (int g = 0; g < sp.nG; g++)
      {
         double o = (-0.5 * (sp.nG - 1) * sp.S + g * sp.S) * FT; // offset (right +)
         double y = -o;
         // skewed support lines: x shifts with offset
         double dx = std::tan(skew) * o;
         GirderLine gl;
         double brgOff = 0.75 * FT, endD = 0.5 * FT;
         double xp1 = x0 + dx, xp2 = x0 + sp.L * FT + dx;
         bool contStart = sp.continuous && i > 0, contEnd = sp.continuous && i + 1 < sp.spans;
         (void)contStart; (void)contEnd;
         gl.pier1 = { xp1, y };
         gl.brg1 = { xp1 + (i == 0 ? 0.0 : brgOff), y };
         gl.end1 = { gl.brg1.x - endD * (i == 0 ? 1 : 0) - (i == 0 ? 0 : -(brgOff - endD)), y };
         if (i == 0) { gl.end1 = { xp1 - endD, y }; gl.brg1 = { xp1, y }; gl.pier1 = gl.brg1; }
         else { gl.end1 = { xp1 + (brgOff - endD), y }; }
         gl.pier2 = { xp2, y };
         if (i + 1 == sp.spans) { gl.brg2 = { xp2, y }; gl.end2 = { xp2 + endD, y }; gl.pier2 = gl.brg2; }
         else { gl.brg2 = { xp2 - brgOff, y }; gl.end2 = { xp2 - (brgOff - endD), y }; }
         gl.section = [=](double) { GirderSection q; q.E = Eg; q.A = A; q.Iv = I; q.Il = Iy; q.J = J; q.ecc = ecc; q.yb = yb; q.height = h; return q; };
         s.girders.push_back(gl);
      }
      b.spans.push_back(s);
   }
   for (int p = 0; p <= sp.spans; p++)
   {
      PierInput pi;
      pi.kind = (p == 0 || p == sp.spans) ? SupportKind::Abutment : (sp.continuous ? SupportKind::ContinuousPier : SupportKind::SimplePier);
      b.piers.push_back(pi);
   }
   b.alignment.eval = [](double s, P2& pt, P2& n) { pt = { s, 0 }; n = { 0, -1 }; };
   b.alignment.inverse = [](const P2& p, double& s, double& o) { s = p.x; o = -p.y; };
   double hw = 0.5 * width * FT, curb = sp.curbIn * FT;
   b.leftDeckEdge = [=](double) { return -hw; };
   b.rightDeckEdge = [=](double) { return hw; };
   b.leftCurb = [=](double) { return -hw + curb; };
   b.rightCurb = [=](double) { return hw - curb; };
   return b;
}

// AASHTO 4.6.2.2 (Type k girders), US units
static void Aashto(const Spec& sp, double& gmInt, double& gvInt, double& gmInt1)
{
   double S = sp.S, L = sp.L, ts = 7.5;
   double n = 5000.0 / 3600.0, I = 260741, A = 789, eg = (54 - 24.73) + 1 + ts / 2;
   double Kg = n * (I + A * eg * eg);
   gmInt1 = 0.06 + std::pow(S / 14, 0.4) * std::pow(S / L, 0.3) * std::pow(Kg / (12 * L * ts * ts * ts), 0.1);
   double gm2 = 0.075 + std::pow(S / 9.5, 0.6) * std::pow(S / L, 0.2) * std::pow(Kg / (12 * L * ts * ts * ts), 0.1);
   gmInt = std::max(gmInt1, gm2);
   double gv1 = 0.36 + S / 25, gv2 = 0.2 + S / 12 - std::pow(S / 35, 2);
   gvInt = std::max(gv1, gv2);
   if (sp.skewDeg > 0)
   {
      double th = sp.skewDeg * 3.14159265358979 / 180;
      double c1 = 0.25 * std::pow(Kg / (12 * L * ts * ts * ts), 0.25) * std::pow(S / L, 0.5);
      double fm = 1 - c1 * std::pow(std::tan(std::min(th, 60 * 3.14159265358979 / 180)), 1.5);
      double fv = 1 + 0.2 * std::pow(12 * L * ts * ts * ts / Kg, 0.3) * std::tan(th);
      gmInt *= fm; gmInt1 *= fm; gvInt *= fv;
   }
}

static void Run(const char* name, const Spec& sp)
{
   BridgeInput b = MakeBridge(sp);
   AnalysisOptions opt;
   opt.keepSurfaces = false;
   Results r = RunRefinedAnalysis(b, opt);
   std::printf("\n=== %s ===\n", name);
   if (!r.ok) { std::printf("FAILED: %s\n", r.error.c_str()); return; }
   std::printf("mesh: %d nodes, %d shells, %d beams, %d equations; %d design lanes; %.1f s\n", r.nodes, r.shells, r.beams, r.equations, r.designLanes, r.seconds);
   for (auto& n : r.notes) std::printf("  note: %s\n", n.c_str());
   double gm, gv, gm1; Aashto(sp, gm, gv, gm1);
   std::printf("  AASHTO interior: gM = %.3f (1 lane %.3f), gV = %.3f\n", gm, gm1, gv);
   std::printf("  span gdr   gM+    gM-    gV    | fatigue gM+   gV\n");
   for (auto& g : r.girders)
      std::printf("  %3d  %c   %.3f  %.3f  %.3f  |        %.3f  %.3f\n", g.span + 1, 'A' + g.girder, g.gMpos, g.gMneg, g.gV, g.gMposF, g.gVF);
   for (auto& p : r.piers) if (p.pier > 0 && p.pier < (int)b.spans.size()) std::printf("  pier %d gdr %c  gM- = %.3f\n", p.pier + 1, 'A' + p.girder, p.gMneg);
   // detail for the first interior girder, span 1
   for (auto& s : r.stations)
      if (s.girder == 2 && (s.action == Action::MomentPos || s.action == Action::Shear || s.action == Action::Reaction) && true)
      {
         const char* a = s.action == Action::MomentPos ? "+M" : s.action == Action::MomentNeg ? "-M" : s.action == Action::Shear ? "V " : "R ";
         std::printf("  S%d G-C %-16s %s  FE %12.1f  line %12.1f  g %.3f  lanes %d\n", s.span + 1, s.label.c_str(), a, s.fe, s.line, s.g, s.governing.lanes);
      }
}

int main()
{
   Spec a; Run("simple span 100 ft, 5 x Type IV @ 8 ft", a);
   Spec b = a; b.skewDeg = 30; Run("same, 30 deg skew", b);
   Spec c = a; c.spans = 2; c.continuous = true; Run("2 x 100 ft continuous", c);
   return 0;
}
