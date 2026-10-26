// Validation of the RfaFem engine against closed-form solutions.
#include "../fem/RfaFem.h"
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

using namespace rfa;

static int g_fail = 0, g_pass = 0;
static void check(const char* name, double got, double expect, double reltol)
{
   double err = std::fabs(got - expect) / std::max(std::fabs(expect), 1e-30);
   bool ok = err <= reltol;
   std::printf("%-58s got %14.6g  expect %14.6g  err %8.3f%%  %s\n", name, got, expect, 100 * err, ok ? "ok" : "FAIL");
   ok ? g_pass++ : g_fail++;
}

// ---------------------------------------------------------------
// 1. Simply supported beam (beam elements only), with eccentricity
// ---------------------------------------------------------------
static void TestBeam(double ecc)
{
   Model m;
   const double L = 20.0, E = 30e9, I = 0.2, A = 0.5;
   const int ne = 20;
   BeamProps bp; bp.E = E; bp.G = E / 2.4; bp.A = A; bp.Iv = I; bp.Il = 0.05; bp.J = 0.01;
   int pr = m.AddBeamProps(bp);
   for (int i = 0; i <= ne; i++) m.AddNode(L * i / ne, 0.0);
   for (int i = 0; i < ne; i++) m.AddBeam(i, i + 1, pr, ecc, ecc);
   // supports: w both ends, v and rx (torsion) both ends, u at start, rz at start
   m.FixDof(0, W); m.FixDof(ne, W);
   m.FixDof(0, V); m.FixDof(ne, V);
   m.FixDof(0, RX); m.FixDof(ne, RX);
   m.FixDof(0, U);
   std::string err;
   if (!m.Factor(&err)) { std::printf("factor failed: %s\n", err.c_str()); g_fail++; return; }
   const double P = 1000.0;
   std::vector<double> f(6 * m.NumNodes(), 0.0);
   f[6 * (ne / 2) + W] = -P;
   auto u = m.Solve(f);
   char nm[128];
   std::snprintf(nm, sizeof nm, "beam e=%.2f mid deflection PL^3/48EI", ecc);
   check(nm, -u[6 * (ne / 2) + W], P * L * L * L / (48 * E * I), 1e-9);
   std::snprintf(nm, sizeof nm, "beam e=%.2f mid moment (elem left, end 1) PL/4", ecc);
   check(nm, m.Evaluate(m.BeamMomentSag(ne / 2 - 1, 1), u), P * L / 4, 1e-9);
   std::snprintf(nm, sizeof nm, "beam e=%.2f mid moment (elem right, end 0) PL/4", ecc);
   check(nm, m.Evaluate(m.BeamMomentSag(ne / 2, 0), u), P * L / 4, 1e-9);
   std::snprintf(nm, sizeof nm, "beam e=%.2f shear left half +P/2", ecc);
   check(nm, m.Evaluate(m.BeamShear(2, 0), u), P / 2, 1e-9);
   std::snprintf(nm, sizeof nm, "beam e=%.2f shear right half -P/2", ecc);
   check(nm, m.Evaluate(m.BeamShear(ne - 2, 1), u), -P / 2, 1e-9);
   std::snprintf(nm, sizeof nm, "beam e=%.2f left reaction (fixed w) +P/2", ecc);
   check(nm, m.Evaluate(m.Reaction(0, W), u), P / 2, 1e-9);

   // influence line of moment at quarter point: value at mid = (L/4)(L/2)/L = L/8
   auto I1 = m.InfluenceSurface(m.BeamMomentSag(ne / 4 - 1, 1));
   std::snprintf(nm, sizeof nm, "beam e=%.2f IL M(L/4) at L/2 = L/8", ecc);
   check(nm, I1[ne / 2], L / 8, 1e-9);
   std::snprintf(nm, sizeof nm, "beam e=%.2f IL M(L/4) at L/4 = 3L/16", ecc);
   check(nm, I1[ne / 4], 3 * L / 16, 1e-9);
   auto IR = m.InfluenceSurface(m.Reaction(0, W));
   std::snprintf(nm, sizeof nm, "beam e=%.2f IL R0 at support = 1", ecc);
   check(nm, IR[0], 1.0, 1e-12);
   std::snprintf(nm, sizeof nm, "beam e=%.2f IL R0 at 3L/4 = 0.25", ecc);
   check(nm, IR[3 * ne / 4], 0.25, 1e-9);
}

// ---------------------------------------------------------------
// 2. Simply supported square plate (Kirchhoff/Navier reference)
// ---------------------------------------------------------------
static void TestPlate(int n, double skewDeg = 0)
{
   Model m;
   const double a = 4.0, t = 0.04, E = 200e9, nu = 0.3; // a/t = 100 (thin)
   ShellProps sp; sp.E = E; sp.nu = nu; sp.t = t;
   int pr = m.AddShellProps(sp);
   auto id = [&](int i, int j) { return j * (n + 1) + i; };
   double sk = std::tan(skewDeg * 3.14159265358979 / 180);
   for (int j = 0; j <= n; j++) for (int i = 0; i <= n; i++)
   {
      // interior lines slanted (parallelogram elements), ramped to square at the edges
      double y = a * j / n, x = a * i / n;
      double r = std::min({ 1.0, std::min(i, n - i) / 4.0, std::min(j, n - j) / 4.0 });
      x += r * sk * (y - a / 2) * 0.25;
      m.AddNode(x, y);
   }
   for (int j = 0; j < n; j++) for (int i = 0; i < n; i++) m.AddShell(id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1), pr);
   for (int j = 0; j <= n; j++) for (int i = 0; i <= n; i++)
   {
      int k = id(i, j);
      bool ex = (i == 0 || i == n), ey = (j == 0 || j == n);
      if (ex || ey) m.FixDof(k, W);
      if (ey) m.FixDof(k, RY);   // hard SS: edge parallel to x -> ry = 0
      if (ex) m.FixDof(k, RX);   // edge parallel to y -> rx = 0
      m.FixDof(k, U); m.FixDof(k, V); m.FixDof(k, RZ); // pure bending test
   }
   std::string err;
   if (!m.Factor(&err)) { std::printf("factor failed: %s\n", err.c_str()); g_fail++; return; }
   const double D = E * t * t * t / (12 * (1 - nu * nu));
   const double P = 1000.0;
   int c = id(n / 2, n / 2);
   std::vector<double> f(6 * m.NumNodes(), 0.0);
   f[6 * c + W] = -P;
   auto u = m.Solve(f);
   char nm[128];
   std::snprintf(nm, sizeof nm, "SS plate %dx%d skew %g centre point load w = 0.01160 Pa^2/D", n, n, skewDeg);
   check(nm, -u[6 * c + W], 0.01160 * P * a * a / D, skewDeg > 0 ? 0.04 : 0.02);

   // uniform load
   const double q = 1000.0;
   std::fill(f.begin(), f.end(), 0.0);
   double h = a / n;
   (void)h;
   for (int j = 0; j < n; j++) for (int i = 0; i < n; i++)
   {
      // consistent load: q * integral of N_k over the element (2x2 Gauss)
      int nd[4] = { id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1) };
      const Vec3& p0 = m.NodePos(nd[0]); const Vec3& p1 = m.NodePos(nd[1]); const Vec3& p2 = m.NodePos(nd[2]); const Vec3& p3 = m.NodePos(nd[3]);
      double X[4] = { p0.x,p1.x,p2.x,p3.x }, Y[4] = { p0.y,p1.y,p2.y,p3.y };
      static const double xs[4] = { -1,1,1,-1 }, es[4] = { -1,-1,1,1 };
      const double g = 0.57735026918962576;
      for (int gi = 0; gi < 4; gi++)
      {
         double xi = g * xs[gi], et = g * es[gi];
         double xr = 0, yr = 0, xe = 0, ye = 0, N[4];
         for (int k = 0; k < 4; k++)
         {
            N[k] = 0.25 * (1 + xi * xs[k]) * (1 + et * es[k]);
            xr += 0.25 * xs[k] * (1 + et * es[k]) * X[k]; yr += 0.25 * xs[k] * (1 + et * es[k]) * Y[k];
            xe += 0.25 * es[k] * (1 + xi * xs[k]) * X[k]; ye += 0.25 * es[k] * (1 + xi * xs[k]) * Y[k];
         }
         double dJ = xr * ye - yr * xe;
         for (int k = 0; k < 4; k++) f[6 * nd[k] + W] -= q * N[k] * dJ;
      }
   }
   u = m.Solve(f);
   std::snprintf(nm, sizeof nm, "SS plate %dx%d skew %g uniform w = 0.00406 qa^4/D", n, n, skewDeg);
   check(nm, -u[6 * c + W], 0.00406 * q * a * a * a * a / D, skewDeg > 0 ? 0.02 : 0.01);
   // Mx at centre: element touching the centre node, evaluate at its corner (xi=1,eta=1)
   int e = (n / 2 - 1) * n + (n / 2 - 1);
   double Mx = m.Evaluate(m.PlateMtt(e, 1, 1, Vec3(1, 0, 0)), u);
   std::snprintf(nm, sizeof nm, "SS plate %dx%d skew %g uniform Mx(centre) = -0.0479 qa^2", n, n, skewDeg);
   check(nm, Mx, -0.0479 * q * a * a, skewDeg > 0 ? 0.05 : 0.03);
   double Qx = m.Evaluate(m.PlateQt(0 * n + 0, -1, 0, Vec3(1, 0, 0)), u); // just exercise; no ref
   (void)Qx;
}

// ---------------------------------------------------------------
// 3. Membrane: cantilever strip under end moment (incompatible modes)
// ---------------------------------------------------------------
static void TestMembrane(bool distort, double drill = 1e-3)
{
   Model m;
   m.SetDrillingFactor(drill);
   const double L = 10, H = 1, t = 0.1, E = 1e6;
   ShellProps sp; sp.E = E; sp.nu = 0.0; sp.t = t;
   int pr = m.AddShellProps(sp);
   const int nx = 10, ny = 2;
   auto id = [&](int i, int j) { return j * (nx + 1) + i; };
   for (int j = 0; j <= ny; j++) for (int i = 0; i <= nx; i++)
   {
      double x = L * i / nx;
      if (distort && i > 0 && i < nx && j == 1) x += (i % 2 ? 0.3 : -0.3);
      if (distort && i > 0 && i < nx && j != 1) x += (i % 2 ? -0.15 : 0.15) * (j == 0 ? 1 : -1);
      m.AddNode(x, H * j / ny);
   }
   for (int j = 0; j < ny; j++) for (int i = 0; i < nx; i++) m.AddShell(id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1), pr);
   for (int j = 0; j <= ny; j++) for (int i = 0; i <= nx; i++)
   {
      int k = id(i, j);
      m.FixDof(k, W); m.FixDof(k, RX); m.FixDof(k, RY);
      if (drill == 0) m.FixDof(k, RZ);
      if (i == 0) { m.FixDof(k, U); m.FixDof(k, V); }
   }
   std::string err;
   if (!m.Factor(&err)) { std::printf("factor failed: %s\n", err.c_str()); g_fail++; return; }
   const double F = 100.0; // couple: +F at top, -F at bottom => M = F*H
   std::vector<double> f(6 * m.NumNodes(), 0.0);
   f[6 * id(nx, ny) + U] = -F;
   f[6 * id(nx, 0) + U] = F;
   auto u = m.Solve(f);
   double I = t * H * H * H / 12, M = F * H;
   check(distort ? "membrane strip (distorted) end moment tip v = ML^2/2EI" : "membrane strip end moment tip v = ML^2/2EI",
      u[6 * id(nx, 1) + V], M * L * L / (2 * E * I), distort ? 0.05 : 1e-3);
   std::printf("   (drill factor %g)\n", drill);
}


// ---------------------------------------------------------------
// 3b. Membrane patch test (uniform tension) on a distorted mesh and
//     cantilever bending with parallelogram (skewed) elements
// ---------------------------------------------------------------
static void TestMembranePatch()
{
   Model m;
   const double E = 1e6, t = 0.1, nu = 0.25;
   ShellProps sp; sp.E = E; sp.nu = nu; sp.t = t;
   int pr = m.AddShellProps(sp);
   const int nx = 4, ny = 3; const double L = 4, H = 3;
   auto id = [&](int i, int j) { return j * (nx + 1) + i; };
   for (int j = 0; j <= ny; j++) for (int i = 0; i <= nx; i++)
   {
      double x = L * i / nx, y = H * j / ny;
      if (i > 0 && i < nx) x += 0.35 * ((i + j) % 2 ? 1 : -1);
      if (j > 0 && j < ny) y += 0.25 * ((i + 2 * j) % 3 - 1);
      m.AddNode(x, y);
   }
   for (int j = 0; j < ny; j++) for (int i = 0; i < nx; i++) m.AddShell(id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1), pr);
   for (int j = 0; j <= ny; j++) for (int i = 0; i <= nx; i++)
   {
      int k = id(i, j); m.FixDof(k, W); m.FixDof(k, RX); m.FixDof(k, RY);
      if (i == 0) m.FixDof(k, U);
   }
   m.FixDof(id(0, 0), V);
   std::string err;
   if (!m.Factor(&err)) { std::printf("factor failed: %s\n", err.c_str()); g_fail++; return; }
   const double sx = 50.0; // uniform stress
   std::vector<double> f(6 * m.NumNodes(), 0.0);
   for (int j = 0; j < ny; j++)
   {
      double h = m.NodePos(id(nx, j + 1)).y - m.NodePos(id(nx, j)).y;
      f[6 * id(nx, j) + U] += 0.5 * sx * t * h;
      f[6 * id(nx, j + 1) + U] += 0.5 * sx * t * h;
   }
   auto u = m.Solve(f);
   check("membrane patch test (distorted) tip u = sL/E", u[6 * id(nx, 2) + U], sx * L / E, 1e-6);
   check("membrane patch test (distorted) Nxx interior = s t", m.Evaluate(m.PlateNtt(5, 0.3, -0.4, Vec3(1, 0, 0)), u), sx * t, 1e-6);
}

static void TestMembraneSkew(double skewDeg)
{
   Model m;
   const double L = 10, H = 1, t = 0.1, E = 1e6;
   ShellProps sp; sp.E = E; sp.nu = 0.0; sp.t = t;
   int pr = m.AddShellProps(sp);
   const int nx = 20, ny = 4;
   double s = std::tan(skewDeg * 3.14159265358979 / 180);
   auto id = [&](int i, int j) { return j * (nx + 1) + i; };
   // skewed (parallelogram) interior; the clamped and loaded ends stay square
   for (int j = 0; j <= ny; j++) for (int i = 0; i <= nx; i++)
   {
      double y = H * j / ny, x = L * i / nx;
      double ramp = std::min(1.0, std::min(i, nx - i) / 4.0); // ramp skew in over 4 columns
      x += ramp * s * (y - H / 2);
      m.AddNode(x, y);
   }
   for (int j = 0; j < ny; j++) for (int i = 0; i < nx; i++) m.AddShell(id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1), pr);
   for (int j = 0; j <= ny; j++) for (int i = 0; i <= nx; i++)
   {
      int k = id(i, j); m.FixDof(k, W); m.FixDof(k, RX); m.FixDof(k, RY);
      if (i == 0) { m.FixDof(k, U); if (j == ny / 2) m.FixDof(k, V); }
   }
   std::string err;
   if (!m.Factor(&err)) { std::printf("factor failed: %s\n", err.c_str()); g_fail++; return; }
   // linear end stress distribution = pure moment M
   const double M = 100.0, I = t * H * H * H / 12;
   std::vector<double> f(6 * m.NumNodes(), 0.0);
   double h = H / ny;
   for (int j = 0; j < ny; j++)
   {
      // consistent nodal forces for sigma = -M y'/I over [y_j, y_j+1], y' from centroid
      double y0 = j * h - H / 2, y1 = y0 + h;
      double s0 = -M * y0 / I * t, s1 = -M * y1 / I * t;
      f[6 * id(nx, j) + U] += h * (2 * s0 + s1) / 6;
      f[6 * id(nx, j + 1) + U] += h * (s0 + 2 * s1) / 6;
   }
   auto u = m.Solve(f);
   char nm[128];
   std::snprintf(nm, sizeof nm, "membrane cantilever, %g deg parallelogram mesh, tip v", skewDeg);
   check(nm, u[6 * id(nx, ny / 2) + V], M * L * L / (2 * E * I), skewDeg < 40 ? 0.02 : 0.05);
}

// ---------------------------------------------------------------
// 4. Composite T-beam: narrow deck strip on one eccentric girder
// ---------------------------------------------------------------
static void TestTBeam()
{
   Model m;
   const double L = 20, b = 1.2, t = 0.2, Ed = 25e9, Eg = 25e9;
   const double Ag = 0.4, Ig = 0.08, e = 0.7; // girder centroid 0.7 m below deck mid-surface
   ShellProps sp; sp.E = Ed; sp.nu = 0.2; sp.t = t;
   int ps = m.AddShellProps(sp);
   BeamProps bp; bp.E = Eg; bp.G = Eg / 2.4; bp.A = Ag; bp.Iv = Ig; bp.Il = 0.01; bp.J = 0.005;
   int pb = m.AddBeamProps(bp);
   const int nx = 40, ny = 4;
   auto id = [&](int i, int j) { return j * (nx + 1) + i; };
   for (int j = 0; j <= ny; j++) for (int i = 0; i <= nx; i++) m.AddNode(L * i / nx, -b / 2 + b * j / ny);
   for (int j = 0; j < ny; j++) for (int i = 0; i < nx; i++) m.AddShell(id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1), ps);
   int gj = ny / 2;
   for (int i = 0; i < nx; i++) m.AddBeam(id(i, gj), id(i + 1, gj), pb, e, e);
   // bearings at girder: vertical at both ends; transverse + longitudinal at the start
   // (longitudinal restraint applied at bearing level by a stiff spring)
   m.FixDof(id(0, gj), W); m.FixDof(id(nx, gj), W);
   m.FixDof(id(0, gj), V); m.FixDof(id(nx, gj), V);
   m.FixDof(id(0, gj), RX); m.FixDof(id(nx, gj), RX); // end torsional restraint (single girder line)
   m.AddSpring(id(0, gj), Vec3(1, 0, 0), Vec3(0, 0, -e), 1e12);
   std::string err;
   if (!m.Factor(&err)) { std::printf("factor failed: %s\n", err.c_str()); g_fail++; return; }

   // composite properties (n = 1), centroid measured downward from deck mid-surface
   double Ad = b * t, Id = b * t * t * t / 12;
   double ybar = (Ag * e) / (Ad + Ag);
   double Ic = Id + Ad * ybar * ybar + Ig + Ag * (e - ybar) * (e - ybar);
   const double P = 1000;
   std::vector<double> f(6 * m.NumNodes(), 0.0);
   f[6 * id(nx / 2, gj) + W] = -P;
   auto u = m.Solve(f);
   // shear deformation of the deck and shear lag make the FE slightly softer
   check("T-beam mid deflection ~ PL^3/48EIc (within 3%)", -u[6 * id(nx / 2, gj) + W], P * L * L * L / (48 * Eg * Ic), 0.03);

   // composite moment at mid: beam M + N*(e - ybar) + plate integral of (-Mxx + Nxx*(-ybar)) across width
   int eb = nx / 2 - 1;
   double Mb = m.Evaluate(m.BeamMomentSag(eb, 1), u);
   double Nb = m.Evaluate(m.BeamAxial(eb, 1), u);
   double Mplate = 0, Nplate = 0;
   for (int j = 0; j < ny; j++)
   {
      int el = j * nx + (nx / 2 - 1);
      // Simpson across element width (eta from -1..1 maps over b/ny)
      double w = b / ny;
      double s = 0, sn = 0;
      double et[3] = { -1, 0, 1 }, wt[3] = { 1.0 / 6, 4.0 / 6, 1.0 / 6 };
      for (int k = 0; k < 3; k++)
      {
         s += wt[k] * m.Evaluate(m.PlateMtt(el, 1, et[k], Vec3(1, 0, 0)), u);
         sn += wt[k] * m.Evaluate(m.PlateNtt(el, 1, et[k], Vec3(1, 0, 0)), u);
      }
      Mplate += -s * w; Nplate += sn * w;
   }
   double Mtot = Mb + Nb * (e - ybar) + Mplate + Nplate * (-ybar);
   check("T-beam composite mid moment = PL/4", Mtot, P * L / 4, 0.01);
   check("T-beam axial equilibrium Nbeam = -Nplate", Nb, -Nplate, 0.02);
   check("T-beam left reaction (fixed w) = P/2", m.Evaluate(m.Reaction(id(0, gj), W), u), P / 2, 1e-6);
   // Reciprocity: adjoint influence value at a deck node equals a direct solve
   auto I = m.InfluenceSurface(m.BeamMomentSag(eb, 1));
   std::vector<double> f2(6 * m.NumNodes(), 0.0);
   int k = id(nx / 3, 1);
   f2[6 * k + W] = -1.0;
   auto u2 = m.Solve(f2);
   check("T-beam adjoint influence == direct solve", I[k], m.Evaluate(m.BeamMomentSag(eb, 1), u2), 1e-8);
}

int main()
{
   TestBeam(0.0);
   TestBeam(0.8);
   TestPlate(8);
   TestPlate(16);
   TestPlate(32);
   TestPlate(16, 45);
   TestPlate(32, 45);
   TestMembrane(false);
   TestMembranePatch();
   TestMembraneSkew(0);
   TestMembraneSkew(30);
   TestMembraneSkew(50);
   TestTBeam();
   std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
   return g_fail ? 1 : 0;
}
