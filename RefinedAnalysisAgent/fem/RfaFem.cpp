///////////////////////////////////////////////////////////////////////
// RfaFem - plate + eccentric beam finite element engine (see RfaFem.h)
///////////////////////////////////////////////////////////////////////
#include "RfaFem.h"
#include <algorithm>
#include <queue>
#include <stdexcept>
#include <cstring>
#include <limits>

namespace rfa
{
   ////////////////////////////////////////////////////////////////////
   // Skyline (profile) LDL^T solver
   ////////////////////////////////////////////////////////////////////
   class SkylineSolver
   {
   public:
      explicit SkylineSolver(const std::vector<int>& first) : m_First(first)
      {
         size_t n = first.size();
         m_Start.resize(n + 1);
         size_t pos = 0;
         for (size_t j = 0; j < n; j++)
         {
            m_Start[j] = pos;
            pos += (j - first[j]) + 1; // rows first[j]..j
         }
         m_Start[n] = pos;
         m_A.assign(pos, 0.0);
      }
      size_t Size() const { return m_First.size(); }
      size_t Storage() const { return m_A.size(); }

      // a(i,j) for i <= j, i >= first[j]
      double& At(int i, int j) { return m_A[m_Start[j] + (i - m_First[j])]; }
      double At(int i, int j) const { return m_A[m_Start[j] + (i - m_First[j])]; }
      void Add(int i, int j, double v)
      {
         if (i > j) std::swap(i, j);
         At(i, j) += v;
      }

      bool Factor(std::string* err)
      {
         int n = (int)Size();
         // diagonal stored at At(j,j); we keep d_j there and L_rj = a(r,j)
         double maxDiag = 0;
         for (int j = 0; j < n; j++) maxDiag = std::max(maxDiag, std::fabs(At(j, j)));
         for (int j = 0; j < n; j++)
         {
            int fj = m_First[j];
            double* colj = &m_A[m_Start[j]] - fj; // colj[i] = a(i,j)
            for (int i = fj + 1; i < j; i++)
            {
               int fi = m_First[i];
               int r0 = std::max(fi, fj);
               const double* coli = &m_A[m_Start[i]] - fi;
               double s = 0;
               for (int r = r0; r < i; r++) s += coli[r] * colj[r];
               colj[i] -= s;
            }
            double d = colj[j];
            for (int r = fj; r < j; r++)
            {
               double g = colj[r];
               double l = g / At(r, r);
               colj[r] = l;
               d -= g * l;
            }
            if (!(d > 1.0e-14 * std::max(maxDiag, 1.0e-300)))
            {
               if (err) *err = "stiffness matrix not positive definite at equation " + std::to_string(j) + " (structure unstable / insufficient supports?)";
               return false;
            }
            colj[j] = d;
         }
         return true;
      }

      void SolveInPlace(std::vector<double>& b) const
      {
         int n = (int)Size();
         for (int j = 0; j < n; j++)
         {
            int fj = m_First[j];
            const double* colj = &m_A[m_Start[j]] - fj;
            double s = 0;
            for (int r = fj; r < j; r++) s += colj[r] * b[r];
            b[j] -= s;
         }
         for (int j = 0; j < n; j++) b[j] /= At(j, j);
         for (int j = n - 1; j >= 0; j--)
         {
            int fj = m_First[j];
            const double* colj = &m_A[m_Start[j]] - fj;
            double bj = b[j];
            for (int r = fj; r < j; r++) b[r] -= colj[r] * bj;
         }
      }

   private:
      std::vector<int> m_First;
      std::vector<size_t> m_Start;
      std::vector<double> m_A;
   };

   ////////////////////////////////////////////////////////////////////
   // small helpers
   ////////////////////////////////////////////////////////////////////
   static const double XI[4] = { -1, 1, 1, -1 };
   static const double ETA[4] = { -1, -1, 1, 1 };
   static const double GP = 0.57735026918962576451;

   static void Shape(double xi, double eta, double N[4], double dNdxi[4], double dNdeta[4])
   {
      for (int i = 0; i < 4; i++)
      {
         N[i] = 0.25 * (1 + xi * XI[i]) * (1 + eta * ETA[i]);
         dNdxi[i] = 0.25 * XI[i] * (1 + eta * ETA[i]);
         dNdeta[i] = 0.25 * ETA[i] * (1 + xi * XI[i]);
      }
   }

   struct Jac
   {
      double xr, yr, xs, ys, det;
      double i11, i12, i21, i22; // inverse
   };

   static Jac Jacobian(const double x[4], const double y[4], const double dNdxi[4], const double dNdeta[4])
   {
      Jac J{};
      J.xr = J.yr = J.xs = J.ys = 0;
      for (int i = 0; i < 4; i++)
      {
         J.xr += dNdxi[i] * x[i]; J.yr += dNdxi[i] * y[i];
         J.xs += dNdeta[i] * x[i]; J.ys += dNdeta[i] * y[i];
      }
      J.det = J.xr * J.ys - J.yr * J.xs;
      // [d/dx; d/dy] = Jinv [d/dxi; d/deta], J = [[xr yr],[xs ys]]
      J.i11 = J.ys / J.det; J.i12 = -J.yr / J.det;
      J.i21 = -J.xs / J.det; J.i22 = J.xr / J.det;
      return J;
   }

   // invert small symmetric positive definite matrix (Gauss-Jordan), n <= 8
   static bool Invert(int n, double* A /*n*n row-major*/)
   {
      std::vector<double> inv(n * n, 0.0);
      for (int i = 0; i < n; i++) inv[i * n + i] = 1;
      std::vector<double> a(A, A + n * n);
      for (int c = 0; c < n; c++)
      {
         int p = c;
         for (int r = c + 1; r < n; r++) if (std::fabs(a[r * n + c]) > std::fabs(a[p * n + c])) p = r;
         if (std::fabs(a[p * n + c]) < 1e-300) return false;
         if (p != c) for (int k = 0; k < n; k++) { std::swap(a[p * n + k], a[c * n + k]); std::swap(inv[p * n + k], inv[c * n + k]); }
         double d = a[c * n + c];
         for (int k = 0; k < n; k++) { a[c * n + k] /= d; inv[c * n + k] /= d; }
         for (int r = 0; r < n; r++) if (r != c)
         {
            double f = a[r * n + c];
            if (f == 0) continue;
            for (int k = 0; k < n; k++) { a[r * n + k] -= f * a[c * n + k]; inv[r * n + k] -= f * inv[c * n + k]; }
         }
      }
      std::copy(inv.begin(), inv.end(), A);
      return true;
   }

   ////////////////////////////////////////////////////////////////////
   // Model definition
   ////////////////////////////////////////////////////////////////////
   Model::Model() {}
   Model::~Model() { delete m_pSolver; }

   int Model::AddNode(double x, double y)
   {
      m_Nodes.emplace_back(x, y, 0.0);
      m_Fixed.resize(6 * m_Nodes.size(), 0);
      m_BucketsBuilt = false;
      return (int)m_Nodes.size() - 1;
   }
   int Model::AddShellProps(const ShellProps& p) { m_ShellProps.push_back(p); return (int)m_ShellProps.size() - 1; }
   int Model::AddBeamProps(const BeamProps& p) { m_BeamProps.push_back(p); return (int)m_BeamProps.size() - 1; }
   int Model::AddShell(int n1, int n2, int n3, int n4, int props)
   {
      Shell s; s.n = { n1,n2,n3,n4 }; s.props = props;
      // enforce counter-clockwise ordering
      const Vec3& a = m_Nodes[n1]; const Vec3& b = m_Nodes[n2]; const Vec3& c = m_Nodes[n3]; const Vec3& d = m_Nodes[n4];
      double area2 = (c - a).cross(d - b).z;
      if (area2 < 0) s.n = { n1,n4,n3,n2 };
      m_Shells.push_back(s);
      m_BucketsBuilt = false;
      return (int)m_Shells.size() - 1;
   }
   int Model::AddBeam(int n1, int n2, int props, double e1, double e2)
   {
      Beam b; b.n = { n1,n2 }; b.props = props; b.ecc = { e1,e2 };
      m_Beams.push_back(b);
      return (int)m_Beams.size() - 1;
   }
   void Model::FixDof(int node, int dof) { m_Fixed[6 * node + dof] = 1; }
   int Model::AddSpring(int node, const Vec3& dir, const Vec3& offset, double k)
   {
      Spring s; s.node = node; s.dir = dir.unit(); s.offset = offset; s.k = k;
      m_Springs.push_back(s);
      return (int)m_Springs.size() - 1;
   }

   int Model::AddRotationalSpring(int node, const Vec3& axis, double k)
   {
      Spring s; s.node = node; s.dir = axis.unit(); s.k = k; s.rotational = true;
      m_Springs.push_back(s);
      return (int)m_Springs.size() - 1;
   }

   ////////////////////////////////////////////////////////////////////
   // Shell element
   ////////////////////////////////////////////////////////////////////
   static void MembraneD(const ShellProps& p, double D[3][3])
   {
      double c = p.E * p.t / (1 - p.nu * p.nu);
      D[0][0] = c; D[0][1] = c * p.nu; D[0][2] = 0;
      D[1][0] = c * p.nu; D[1][1] = c; D[1][2] = 0;
      D[2][0] = 0; D[2][1] = 0; D[2][2] = c * (1 - p.nu) / 2;
   }
   static void BendingD(const ShellProps& p, double D[3][3])
   {
      double c = p.E * p.t * p.t * p.t / (12 * (1 - p.nu * p.nu));
      D[0][0] = c; D[0][1] = c * p.nu; D[0][2] = 0;
      D[1][0] = c * p.nu; D[1][1] = c; D[1][2] = 0;
      D[2][0] = 0; D[2][1] = 0; D[2][2] = c * (1 - p.nu) / 2;
   }

   // Membrane: compatible B (3x8, dofs u1 v1 u2 v2 ...) and incompatible B (3x4: a1u a2u a1v a2v)
   static void MembraneB(const double x[4], const double y[4], double xi, double eta, const Jac& J0, double detJ0,
      double Bc[3][8], double Bi[3][4], double& detJ)
   {
      double N[4], dr[4], ds[4];
      Shape(xi, eta, N, dr, ds);
      Jac J = Jacobian(x, y, dr, ds);
      detJ = J.det;
      for (int i = 0; i < 4; i++)
      {
         double Nx = J.i11 * dr[i] + J.i12 * ds[i];
         double Ny = J.i21 * dr[i] + J.i22 * ds[i];
         Bc[0][2 * i] = Nx; Bc[0][2 * i + 1] = 0;
         Bc[1][2 * i] = 0;  Bc[1][2 * i + 1] = Ny;
         Bc[2][2 * i] = Ny; Bc[2][2 * i + 1] = Nx;
      }
      // incompatible modes P1 = 1-xi^2, P2 = 1-eta^2 ; derivatives via J0 and scaled (Taylor)
      double sc = detJ0 / detJ;
      double P1r = -2 * xi, P1s = 0, P2r = 0, P2s = -2 * eta;
      double P1x = (J0.i11 * P1r + J0.i12 * P1s) * sc, P1y = (J0.i21 * P1r + J0.i22 * P1s) * sc;
      double P2x = (J0.i11 * P2r + J0.i12 * P2s) * sc, P2y = (J0.i21 * P2r + J0.i22 * P2s) * sc;
      // order: a1 (u,P1), a2 (u,P2), a3 (v,P1), a4 (v,P2)
      Bi[0][0] = P1x; Bi[0][1] = P2x; Bi[0][2] = 0;   Bi[0][3] = 0;
      Bi[1][0] = 0;   Bi[1][1] = 0;   Bi[1][2] = P1y; Bi[1][3] = P2y;
      Bi[2][0] = P1y; Bi[2][1] = P2y; Bi[2][2] = P1x; Bi[2][3] = P2x;
   }

   static void MembraneMatrices(const double x[4], const double y[4], const ShellProps& p,
      double Kcc[8][8], double Kci[8][4], double Kii[4][4])
   {
      double D[3][3]; MembraneD(p, D);
      double N[4], dr[4], ds[4];
      Shape(0, 0, N, dr, ds);
      Jac J0 = Jacobian(x, y, dr, ds);
      std::memset(Kcc, 0, sizeof(double) * 64); std::memset(Kci, 0, sizeof(double) * 32); std::memset(Kii, 0, sizeof(double) * 16);
      for (int gi = 0; gi < 2; gi++) for (int gj = 0; gj < 2; gj++)
      {
         double xi = gi ? GP : -GP, eta = gj ? GP : -GP;
         double Bc[3][8], Bi[3][4], detJ;
         MembraneB(x, y, xi, eta, J0, J0.det, Bc, Bi, detJ);
         double DBc[3][8], DBi[3][4];
         for (int r = 0; r < 3; r++)
         {
            for (int c = 0; c < 8; c++) { DBc[r][c] = 0; for (int k = 0; k < 3; k++) DBc[r][c] += D[r][k] * Bc[k][c]; }
            for (int c = 0; c < 4; c++) { DBi[r][c] = 0; for (int k = 0; k < 3; k++) DBi[r][c] += D[r][k] * Bi[k][c]; }
         }
         for (int a = 0; a < 8; a++)
         {
            for (int b = 0; b < 8; b++) { double s = 0; for (int k = 0; k < 3; k++) s += Bc[k][a] * DBc[k][b]; Kcc[a][b] += s * detJ; }
            for (int b = 0; b < 4; b++) { double s = 0; for (int k = 0; k < 3; k++) s += Bc[k][a] * DBi[k][b]; Kci[a][b] += s * detJ; }
         }
         for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++)
         {
            double s = 0; for (int k = 0; k < 3; k++) s += Bi[k][a] * DBi[k][b]; Kii[a][b] += s * detJ;
         }
      }
   }

   // MITC4 bending (3x12) and shear (2x12) B matrices; dofs per node (w, rx, ry)
   static void MITC4B(const double x[4], const double y[4], double xi, double eta, double Bb[3][12], double Bs[2][12], double& detJ)
   {
      double N[4], dr[4], ds[4];
      Shape(xi, eta, N, dr, ds);
      Jac J = Jacobian(x, y, dr, ds);
      detJ = J.det;
      for (int i = 0; i < 4; i++)
      {
         double Nx = J.i11 * dr[i] + J.i12 * ds[i];
         double Ny = J.i21 * dr[i] + J.i22 * ds[i];
         // kx = d(ry)/dx ; ky = -d(rx)/dy ; kxy = d(ry)/dy - d(rx)/dx
         Bb[0][3 * i] = 0; Bb[0][3 * i + 1] = 0;   Bb[0][3 * i + 2] = Nx;
         Bb[1][3 * i] = 0; Bb[1][3 * i + 1] = -Ny; Bb[1][3 * i + 2] = 0;
         Bb[2][3 * i] = 0; Bb[2][3 * i + 1] = -Nx; Bb[2][3 * i + 2] = Ny;
      }
      // covariant transverse shear at tying points
      // g_r = dw/dxi + ry*x_xi - rx*y_xi ; g_s = dw/deta + ry*x_eta - rx*y_eta
      auto covRow = [&](double xt, double et, bool rdir, double row[12])
      {
         double Nt[4], drt[4], dst[4];
         Shape(xt, et, Nt, drt, dst);
         Jac Jt = Jacobian(x, y, drt, dst);
         double xd = rdir ? Jt.xr : Jt.xs;
         double yd = rdir ? Jt.yr : Jt.ys;
         for (int i = 0; i < 4; i++)
         {
            row[3 * i] = rdir ? drt[i] : dst[i];
            row[3 * i + 1] = -Nt[i] * yd;
            row[3 * i + 2] = Nt[i] * xd;
         }
      };
      double rA[12], rC[12], sB[12], sD[12];
      covRow(0, -1, true, rA);
      covRow(0, 1, true, rC);
      covRow(-1, 0, false, sB);
      covRow(1, 0, false, sD);
      double gr[12], gs[12];
      for (int k = 0; k < 12; k++)
      {
         gr[k] = 0.5 * (1 - eta) * rA[k] + 0.5 * (1 + eta) * rC[k];
         gs[k] = 0.5 * (1 - xi) * sB[k] + 0.5 * (1 + xi) * sD[k];
      }
      // [gx;gy] = Jinv [gr;gs]
      for (int k = 0; k < 12; k++)
      {
         Bs[0][k] = J.i11 * gr[k] + J.i12 * gs[k];
         Bs[1][k] = J.i21 * gr[k] + J.i22 * gs[k];
      }
   }

   void Model::ShellStiffness(int e, double K[24][24]) const
   {
      const Shell& s = m_Shells[e];
      const ShellProps& p = m_ShellProps[s.props];
      double x[4], y[4];
      for (int i = 0; i < 4; i++) { x[i] = m_Nodes[s.n[i]].x; y[i] = m_Nodes[s.n[i]].y; }
      std::memset(K, 0, sizeof(double) * 24 * 24);

      // --- membrane (condensed incompatible modes) ---
      double Kcc[8][8], Kci[8][4], Kii[4][4];
      MembraneMatrices(x, y, p, Kcc, Kci, Kii);
      double Kinv[16]; for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++) Kinv[a * 4 + b] = Kii[a][b];
      Invert(4, Kinv);
      double Km[8][8];
      for (int a = 0; a < 8; a++) for (int b = 0; b < 8; b++)
      {
         double s2 = 0;
         for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) s2 += Kci[a][i] * Kinv[i * 4 + j] * Kci[b][j];
         Km[a][b] = Kcc[a][b] - s2;
      }
      static const int mmap[8] = { 0,1, 6,7, 12,13, 18,19 };
      for (int a = 0; a < 8; a++) for (int b = 0; b < 8; b++) K[mmap[a]][mmap[b]] += Km[a][b];

      // --- bending + shear (MITC4) ---
      double Db[3][3]; BendingD(p, Db);
      double kappa = 5.0 / 6.0, G = p.E / (2 * (1 + p.nu));
      double Ds = kappa * G * p.t;
      static const int bmap[12] = { 2,3,4, 8,9,10, 14,15,16, 20,21,22 };
      for (int gi = 0; gi < 2; gi++) for (int gj = 0; gj < 2; gj++)
      {
         double xi = gi ? GP : -GP, eta = gj ? GP : -GP;
         double Bb[3][12], Bs[2][12], detJ;
         MITC4B(x, y, xi, eta, Bb, Bs, detJ);
         for (int a = 0; a < 12; a++) for (int b = 0; b < 12; b++)
         {
            double s2 = 0;
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) s2 += Bb[r][a] * Db[r][c] * Bb[c][b];
            s2 += Ds * (Bs[0][a] * Bs[0][b] + Bs[1][a] * Bs[1][b]);
            K[bmap[a]][bmap[b]] += s2 * detJ;
         }
      }

      // --- drilling: alpha*G*t * integral (omega - rz)^2 ---
      double kd = m_DrillFactor * G * p.t;
      for (int gi = 0; gi < 2; gi++) for (int gj = 0; gj < 2; gj++)
      {
         double xi = gi ? GP : -GP, eta = gj ? GP : -GP;
         double N[4], dr[4], ds[4];
         Shape(xi, eta, N, dr, ds);
         Jac J = Jacobian(x, y, dr, ds);
         double b[24] = { 0 };
         for (int i = 0; i < 4; i++)
         {
            double Nx = J.i11 * dr[i] + J.i12 * ds[i];
            double Ny = J.i21 * dr[i] + J.i22 * ds[i];
            b[6 * i + 0] += -0.5 * Ny; // omega = 0.5(dv/dx - du/dy)
            b[6 * i + 1] += 0.5 * Nx;
            b[6 * i + 5] += -N[i];
         }
         for (int a = 0; a < 24; a++) if (b[a] != 0) for (int c = 0; c < 24; c++) if (b[c] != 0) K[a][c] += kd * b[a] * b[c] * J.det;
      }
   }

   ////////////////////////////////////////////////////////////////////
   // Beam element
   ////////////////////////////////////////////////////////////////////
   void Model::BeamTransform(int e, double T[12][12], double& L) const
   {
      const Beam& b = m_Beams[e];
      Vec3 P1 = m_Nodes[b.n[0]] + Vec3(0, 0, -b.ecc[0]);
      Vec3 P2 = m_Nodes[b.n[1]] + Vec3(0, 0, -b.ecc[1]);
      Vec3 ex = (P2 - P1);
      L = ex.norm();
      ex = ex.unit();
      Vec3 ey = Vec3(0, 0, 1).cross(ex).unit();
      Vec3 ez = ex.cross(ey);
      double R[3][3] = { {ex.x,ex.y,ex.z},{ey.x,ey.y,ey.z},{ez.x,ez.y,ez.z} };
      std::memset(T, 0, sizeof(double) * 144);
      for (int end = 0; end < 2; end++)
      {
         double e2 = b.ecc[end];
         // Tn (6x6): u_P = u - e*ry ; v_P = v + e*rx ; w_P = w ; rotations equal
         double Tn[6][6] = { {0} };
         for (int i = 0; i < 6; i++) Tn[i][i] = 1;
         Tn[0][4] = -e2;
         Tn[1][3] = e2;
         int o = 6 * end;
         for (int i = 0; i < 3; i++) for (int j = 0; j < 6; j++)
         {
            double st = 0, sr = 0;
            for (int k = 0; k < 3; k++) { st += R[i][k] * Tn[k][j]; sr += R[i][k] * Tn[3 + k][j]; }
            T[o + i][o + j] = st;
            T[o + 3 + i][o + j] = sr;
         }
      }
   }

   void Model::BeamLocalStiffness(int e, double k[12][12], double L) const
   {
      const BeamProps& p = m_BeamProps[m_Beams[e].props];
      std::memset(k, 0, sizeof(double) * 144);
      double EA = p.E * p.A / L, GJ = p.G * p.J / L;
      double Iz = p.Il, Iy = p.Iv, E = p.E;
      double L2 = L * L, L3 = L2 * L;
      k[0][0] = EA; k[0][6] = -EA; k[6][6] = EA;
      k[3][3] = GJ; k[3][9] = -GJ; k[9][9] = GJ;
      // x'-y' plane (about z', lateral)
      k[1][1] = 12 * E * Iz / L3; k[1][5] = 6 * E * Iz / L2; k[1][7] = -12 * E * Iz / L3; k[1][11] = 6 * E * Iz / L2;
      k[5][5] = 4 * E * Iz / L; k[5][7] = -6 * E * Iz / L2; k[5][11] = 2 * E * Iz / L;
      k[7][7] = 12 * E * Iz / L3; k[7][11] = -6 * E * Iz / L2; k[11][11] = 4 * E * Iz / L;
      // x'-z' plane (about y', vertical)
      k[2][2] = 12 * E * Iy / L3; k[2][4] = -6 * E * Iy / L2; k[2][8] = -12 * E * Iy / L3; k[2][10] = -6 * E * Iy / L2;
      k[4][4] = 4 * E * Iy / L; k[4][8] = 6 * E * Iy / L2; k[4][10] = 2 * E * Iy / L;
      k[8][8] = 12 * E * Iy / L3; k[8][10] = 6 * E * Iy / L2; k[10][10] = 4 * E * Iy / L;
      for (int i = 0; i < 12; i++) for (int j = 0; j < i; j++) k[i][j] = k[j][i];
   }

   ////////////////////////////////////////////////////////////////////
   // Equation numbering (reverse Cuthill-McKee on nodes)
   ////////////////////////////////////////////////////////////////////
   void Model::NumberEquations()
   {
      size_t nn = m_Nodes.size();
      std::vector<std::vector<int>> adj(nn);
      auto link = [&](int a, int b) { if (a != b) { adj[a].push_back(b); adj[b].push_back(a); } };
      for (auto& s : m_Shells) for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) link(s.n[i], s.n[j]);
      for (auto& b : m_Beams) link(b.n[0], b.n[1]);
      for (auto& a : adj) { std::sort(a.begin(), a.end()); a.erase(std::unique(a.begin(), a.end()), a.end()); }

      std::vector<int> order; order.reserve(nn);
      std::vector<char> seen(nn, 0);
      auto bfs = [&](int start, std::vector<int>& out, std::vector<int>* level)
      {
         std::vector<char> vis(nn, 0);
         std::queue<int> q; q.push(start); vis[start] = 1;
         out.clear();
         std::vector<int> lev(nn, 0);
         while (!q.empty())
         {
            int v = q.front(); q.pop(); out.push_back(v);
            std::vector<int> nb;
            for (int w : adj[v]) if (!vis[w]) { vis[w] = 1; nb.push_back(w); lev[w] = lev[v] + 1; }
            std::sort(nb.begin(), nb.end(), [&](int a, int b) { return adj[a].size() < adj[b].size(); });
            for (int w : nb) q.push(w);
         }
         if (level) *level = lev;
      };
      for (size_t s = 0; s < nn; s++)
      {
         if (seen[s]) continue;
         // pseudo-peripheral start: min degree node of component, then farthest
         std::vector<int> comp; bfs((int)s, comp, nullptr);
         int start = comp[0];
         for (int v : comp) if (adj[v].size() < adj[start].size()) start = v;
         std::vector<int> lev;
         for (int it = 0; it < 2; it++)
         {
            std::vector<int> tmp; bfs(start, tmp, &lev);
            int far = tmp.back();
            if (lev[far] <= lev[start]) break;
            start = far;
         }
         std::vector<int> cm; bfs(start, cm, nullptr);
         for (int v : cm) seen[v] = 1;
         order.insert(order.end(), cm.rbegin(), cm.rend());
      }

      m_EqNum.assign(6 * nn, -1);
      int eq = 0;
      for (int nd : order) for (int d = 0; d < 6; d++)
         if (!m_Fixed[6 * nd + d]) m_EqNum[6 * nd + d] = eq++;
      m_NumEq = eq;
   }

   ////////////////////////////////////////////////////////////////////
   // Assembly / factorization
   ////////////////////////////////////////////////////////////////////
   bool Model::Factor(std::string* error)
   {
      NumberEquations();
      int n = m_NumEq;
      std::vector<int> first(n);
      for (int i = 0; i < n; i++) first[i] = i;
      auto touch = [&](const std::vector<int>& gd)
      {
         int mn = std::numeric_limits<int>::max();
         for (int g : gd) { int q = m_EqNum[g]; if (q >= 0) mn = std::min(mn, q); }
         for (int g : gd) { int q = m_EqNum[g]; if (q >= 0) first[q] = std::min(first[q], mn); }
      };
      std::vector<int> gd;
      for (auto& s : m_Shells) { gd.clear(); for (int i = 0; i < 4; i++) for (int d = 0; d < 6; d++) gd.push_back(6 * s.n[i] + d); touch(gd); }
      for (auto& b : m_Beams) { gd.clear(); for (int i = 0; i < 2; i++) for (int d = 0; d < 6; d++) gd.push_back(6 * b.n[i] + d); touch(gd); }
      for (auto& sp : m_Springs) { gd.clear(); for (int d = 0; d < 6; d++) gd.push_back(6 * sp.node + d); touch(gd); }

      delete m_pSolver;
      m_pSolver = new SkylineSolver(first);
      m_ReactionRows.clear();

      auto scatter = [&](const std::vector<int>& g, const double* Ke, int ne)
      {
         for (int a = 0; a < ne; a++)
         {
            int qa = m_EqNum[g[a]];
            if (qa < 0)
            {
               auto& row = m_ReactionRows[g[a]];
               for (int b = 0; b < ne; b++) if (m_EqNum[g[b]] >= 0 && Ke[a * ne + b] != 0) row[g[b]] += Ke[a * ne + b];
               continue;
            }
            for (int b = 0; b < ne; b++)
            {
               int qb = m_EqNum[g[b]];
               if (qb < qa) continue; // fixed (-1) or lower triangle (added from the other row)
               m_pSolver->Add(qa, qb, Ke[a * ne + b]);
            }
         }
      };

      for (int e = 0; e < (int)m_Shells.size(); e++)
      {
         double K[24][24];
         ShellStiffness(e, K);
         gd.clear(); for (int i = 0; i < 4; i++) for (int d = 0; d < 6; d++) gd.push_back(6 * m_Shells[e].n[i] + d);
         scatter(gd, &K[0][0], 24);
      }
      for (int e = 0; e < (int)m_Beams.size(); e++)
      {
         double T[12][12], k[12][12], L;
         BeamTransform(e, T, L);
         BeamLocalStiffness(e, k, L);
         double kT[12][12], K[12][12];
         for (int i = 0; i < 12; i++) for (int j = 0; j < 12; j++) { double s = 0; for (int m = 0; m < 12; m++) s += k[i][m] * T[m][j]; kT[i][j] = s; }
         for (int i = 0; i < 12; i++) for (int j = 0; j < 12; j++) { double s = 0; for (int m = 0; m < 12; m++) s += T[m][i] * kT[m][j]; K[i][j] = s; }
         gd.clear(); for (int i = 0; i < 2; i++) for (int d = 0; d < 6; d++) gd.push_back(6 * m_Beams[e].n[i] + d);
         scatter(gd, &K[0][0], 12);
      }
      for (auto& sp : m_Springs)
      {
         Vec3 rd = sp.offset.cross(sp.dir);
         double a[6] = { sp.dir.x,sp.dir.y,sp.dir.z,rd.x,rd.y,rd.z };
         if (sp.rotational) { a[0] = a[1] = a[2] = 0; a[3] = sp.dir.x; a[4] = sp.dir.y; a[5] = sp.dir.z; }
         double K[6][6];
         for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) K[i][j] = sp.k * a[i] * a[j];
         gd.clear(); for (int d = 0; d < 6; d++) gd.push_back(6 * sp.node + d);
         scatter(gd, &K[0][0], 6);
      }
      std::string err;
      if (!m_pSolver->Factor(&err))
      {
         // map equation back to node/dof for a useful message
         if (error) *error = err;
         return false;
      }
      return true;
   }

   std::vector<double> Model::Solve(const std::vector<double>& f) const
   {
      std::vector<double> b(m_NumEq, 0.0);
      for (size_t g = 0; g < f.size(); g++) { int q = m_EqNum[g]; if (q >= 0) b[q] = f[g]; }
      m_pSolver->SolveInPlace(b);
      std::vector<double> u(6 * m_Nodes.size(), 0.0);
      for (size_t g = 0; g < u.size(); g++) { int q = m_EqNum[g]; if (q >= 0) u[g] = b[q]; }
      return u;
   }

   std::vector<double> Model::SolvePointLoad(double x, double y, double P) const
   {
      std::vector<double> f(6 * m_Nodes.size(), 0.0);
      PointLoc loc = Locate(x, y);
      if (loc.shell < 0) throw std::runtime_error("load point outside the deck");
      double N[4]; ShellShape(loc, N);
      for (int i = 0; i < 4; i++) f[6 * m_Shells[loc.shell].n[i] + W] -= P * N[i];
      return Solve(f);
   }

   ////////////////////////////////////////////////////////////////////
   // Functionals
   ////////////////////////////////////////////////////////////////////
   double Model::Evaluate(const Functional& f, const std::vector<double>& u) const
   {
      double s = 0;
      for (auto& kv : f.c) s += kv.second * u[kv.first];
      return s;
   }

   Functional Model::BeamEndForce(int e, int end, int comp) const
   {
      double T[12][12], k[12][12], L;
      BeamTransform(e, T, L);
      BeamLocalStiffness(e, k, L);
      int r = 6 * end + comp;
      Functional f;
      for (int j = 0; j < 12; j++)
      {
         double s = 0;
         for (int m = 0; m < 12; m++) s += k[r][m] * T[m][j];
         if (s != 0) f.add(6 * m_Beams[e].n[j / 6] + (j % 6), s);
      }
      return f;
   }

   Functional Model::ShellNodalForce(int e, int localNode, int dof) const
   {
      double K[24][24];
      ShellStiffness(e, K);
      Functional f;
      int r = 6 * localNode + dof;
      for (int j = 0; j < 24; j++) if (K[r][j] != 0) f.add(6 * m_Shells[e].n[j / 6] + (j % 6), K[r][j]);
      return f;
   }

   Functional Model::BeamNodalForce(int e, int localNode, int dof) const
   {
      double T[12][12], k[12][12], L;
      BeamTransform(e, T, L);
      BeamLocalStiffness(e, k, L);
      int r = 6 * localNode + dof;
      Functional f;
      for (int j = 0; j < 12; j++)
      {
         // (T^T k T)[r][j]
         double s = 0;
         for (int a = 0; a < 12; a++)
         {
            if (T[a][r] == 0) continue;
            double kt = 0;
            for (int b = 0; b < 12; b++) kt += k[a][b] * T[b][j];
            s += T[a][r] * kt;
         }
         if (s != 0) f.add(6 * m_Beams[e].n[j / 6] + (j % 6), s);
      }
      return f;
   }

   Functional Model::BeamAxial(int e, int end) const
   {
      Functional f = BeamEndForce(e, end, 0);
      if (end == 0) { Functional g; g.add(f, -1.0); return g; }
      return f;
   }
   Functional Model::BeamShear(int e, int end) const
   {
      Functional f = BeamEndForce(e, end, 2);
      if (end == 1) { Functional g; g.add(f, -1.0); return g; }
      return f;
   }
   Functional Model::BeamMomentSag(int e, int end) const
   {
      Functional f = BeamEndForce(e, end, 4);
      if (end == 1) { Functional g; g.add(f, -1.0); return g; }
      return f;
   }

   Functional Model::Reaction(int node, int dof) const
   {
      Functional f;
      int g = 6 * node + dof;
      auto it = m_ReactionRows.find(g);
      if (it != m_ReactionRows.end()) for (auto& kv : it->second) f.add(kv.first, kv.second);
      if (dof == W) f.direct[g] = 1.0; // downward unit load placed on the support itself
      return f;
   }

   Functional Model::SpringForce(int s) const
   {
      const Spring& sp = m_Springs[s];
      Vec3 rd = sp.offset.cross(sp.dir);
      double a[6] = { sp.dir.x,sp.dir.y,sp.dir.z,rd.x,rd.y,rd.z };
      if (sp.rotational) { a[0] = a[1] = a[2] = 0; a[3] = sp.dir.x; a[4] = sp.dir.y; a[5] = sp.dir.z; }
      Functional f;
      for (int d = 0; d < 6; d++) if (a[d] != 0) f.add(6 * sp.node + d, -sp.k * a[d]);
      return f;
   }

   void Model::ShellBendingRecovery(int e, double xi, double eta, double Bb[3][12], double Bs[2][12]) const
   {
      const Shell& s = m_Shells[e];
      double x[4], y[4];
      for (int i = 0; i < 4; i++) { x[i] = m_Nodes[s.n[i]].x; y[i] = m_Nodes[s.n[i]].y; }
      double detJ;
      MITC4B(x, y, xi, eta, Bb, Bs, detJ);
   }

   void Model::ShellMembraneRecovery(int e, double xi, double eta, double B[3][8]) const
   {
      const Shell& s = m_Shells[e];
      const ShellProps& p = m_ShellProps[s.props];
      double x[4], y[4];
      for (int i = 0; i < 4; i++) { x[i] = m_Nodes[s.n[i]].x; y[i] = m_Nodes[s.n[i]].y; }
      double Kcc[8][8], Kci[8][4], Kii[4][4];
      MembraneMatrices(x, y, p, Kcc, Kci, Kii);
      double Kinv[16]; for (int a = 0; a < 4; a++) for (int b = 0; b < 4; b++) Kinv[a * 4 + b] = Kii[a][b];
      Invert(4, Kinv);
      double N[4], dr[4], ds[4];
      Shape(0, 0, N, dr, ds);
      Jac J0 = Jacobian(x, y, dr, ds);
      double Bc[3][8], Bi[3][4], detJ;
      MembraneB(x, y, xi, eta, J0, J0.det, Bc, Bi, detJ);
      // alpha = -Kii^-1 Kic uc  => B = Bc - Bi Kii^-1 Kic
      double A[4][8];
      for (int i = 0; i < 4; i++) for (int c = 0; c < 8; c++) { double s2 = 0; for (int j = 0; j < 4; j++) s2 += Kinv[i * 4 + j] * Kci[c][j]; A[i][c] = s2; }
      for (int r = 0; r < 3; r++) for (int c = 0; c < 8; c++)
      {
         double s2 = 0; for (int i = 0; i < 4; i++) s2 += Bi[r][i] * A[i][c];
         B[r][c] = Bc[r][c] - s2;
      }
   }

   Functional Model::PlateMtt(int e, double xi, double eta, const Vec3& t) const
   {
      const Shell& s = m_Shells[e];
      double Db[3][3]; BendingD(m_ShellProps[s.props], Db);
      double Bb[3][12], Bs[2][12];
      ShellBendingRecovery(e, xi, eta, Bb, Bs);
      double w3[3] = { t.x * t.x, t.y * t.y, 2 * t.x * t.y };
      Functional f;
      for (int k = 0; k < 12; k++)
      {
         double v = 0;
         for (int r = 0; r < 3; r++) { double m = 0; for (int c = 0; c < 3; c++) m += Db[r][c] * Bb[c][k]; v += w3[r] * m; }
         if (v != 0) f.add(6 * s.n[k / 3] + 2 + (k % 3), v);
      }
      return f;
   }

   Functional Model::PlateNtt(int e, double xi, double eta, const Vec3& t) const
   {
      const Shell& s = m_Shells[e];
      double D[3][3]; MembraneD(m_ShellProps[s.props], D);
      double B[3][8];
      ShellMembraneRecovery(e, xi, eta, B);
      double w3[3] = { t.x * t.x, t.y * t.y, 2 * t.x * t.y };
      Functional f;
      for (int k = 0; k < 8; k++)
      {
         double v = 0;
         for (int r = 0; r < 3; r++) { double m = 0; for (int c = 0; c < 3; c++) m += D[r][c] * B[c][k]; v += w3[r] * m; }
         if (v != 0) f.add(6 * s.n[k / 2] + (k % 2), v);
      }
      return f;
   }

   Functional Model::PlateQt(int e, double xi, double eta, const Vec3& t) const
   {
      const Shell& s = m_Shells[e];
      const ShellProps& p = m_ShellProps[s.props];
      double Ds = 5.0 / 6.0 * p.E / (2 * (1 + p.nu)) * p.t;
      double Bb[3][12], Bs[2][12];
      ShellBendingRecovery(e, xi, eta, Bb, Bs);
      Functional f;
      for (int k = 0; k < 12; k++)
      {
         double v = Ds * (t.x * Bs[0][k] + t.y * Bs[1][k]);
         if (v != 0) f.add(6 * s.n[k / 3] + 2 + (k % 3), v);
      }
      return f;
   }

   ////////////////////////////////////////////////////////////////////
   // Influence surfaces
   ////////////////////////////////////////////////////////////////////
   std::vector<std::vector<double>> Model::InfluenceSurfaces(const std::vector<Functional>& fs) const
   {
      std::vector<std::vector<double>> out;
      out.reserve(fs.size());
      size_t nn = m_Nodes.size();
      std::vector<double> b(m_NumEq);
      for (auto& f : fs)
      {
         std::fill(b.begin(), b.end(), 0.0);
         for (auto& kv : f.c) { int q = m_EqNum[kv.first]; if (q >= 0) b[q] += kv.second; }
         m_pSolver->SolveInPlace(b);
         std::vector<double> I(nn, 0.0);
         for (size_t n = 0; n < nn; n++)
         {
            int g = (int)(6 * n + W);
            int q = m_EqNum[g];
            if (q >= 0) I[n] = -b[q];
            else
            {
               // load directly on a support: only that support's reaction sees it
               auto it = f.direct.find(g);
               I[n] = (it == f.direct.end() ? 0.0 : it->second);
            }
         }
         out.push_back(std::move(I));
      }
      return out;
   }

   std::vector<double> Model::InfluenceSurface(const Functional& f) const
   {
      return InfluenceSurfaces({ f }).front();
   }

   ////////////////////////////////////////////////////////////////////
   // Point location
   ////////////////////////////////////////////////////////////////////
   void Model::BuildBuckets() const
   {
      Model* self = const_cast<Model*>(this);
      if (m_Shells.empty()) return;
      double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
      for (auto& p : m_Nodes) { x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); }
      int n = (int)std::sqrt((double)m_Shells.size()) + 1;
      self->m_BX0 = x0; self->m_BY0 = y0;
      self->m_BNX = n; self->m_BNY = n;
      self->m_BDX = std::max((x1 - x0) / n, 1e-9);
      self->m_BDY = std::max((y1 - y0) / n, 1e-9);
      self->m_Buckets.assign(n * n, {});
      for (int e = 0; e < (int)m_Shells.size(); e++)
      {
         double a0 = 1e300, b0 = 1e300, a1 = -1e300, b1 = -1e300;
         for (int i = 0; i < 4; i++) { auto& p = m_Nodes[m_Shells[e].n[i]]; a0 = std::min(a0, p.x); b0 = std::min(b0, p.y); a1 = std::max(a1, p.x); b1 = std::max(b1, p.y); }
         int i0 = std::clamp((int)((a0 - x0) / m_BDX), 0, n - 1), i1 = std::clamp((int)((a1 - x0) / m_BDX), 0, n - 1);
         int j0 = std::clamp((int)((b0 - y0) / m_BDY), 0, n - 1), j1 = std::clamp((int)((b1 - y0) / m_BDY), 0, n - 1);
         for (int i = i0; i <= i1; i++) for (int j = j0; j <= j1; j++) self->m_Buckets[j * n + i].push_back(e);
      }
      m_BucketsBuilt = true;
   }

   PointLoc Model::Locate(double px, double py) const
   {
      if (!m_BucketsBuilt) BuildBuckets();
      PointLoc best;
      int i = (int)std::floor((px - m_BX0) / m_BDX), j = (int)std::floor((py - m_BY0) / m_BDY);
      if (i < 0 || j < 0 || i >= m_BNX || j >= m_BNY)
      {
         // allow points exactly on the max boundary
         if (i == m_BNX) i--; if (j == m_BNY) j--;
         if (i < 0 || j < 0 || i >= m_BNX || j >= m_BNY) return best;
      }
      double bestOut = 1e300;
      for (int e : m_Buckets[j * m_BNX + i])
      {
         const Shell& s = m_Shells[e];
         double x[4], y[4];
         for (int k = 0; k < 4; k++) { x[k] = m_Nodes[s.n[k]].x; y[k] = m_Nodes[s.n[k]].y; }
         double xi = 0, eta = 0;
         for (int it = 0; it < 20; it++)
         {
            double N[4], dr[4], ds[4];
            Shape(xi, eta, N, dr, ds);
            double fx = -px, fy = -py;
            for (int k = 0; k < 4; k++) { fx += N[k] * x[k]; fy += N[k] * y[k]; }
            Jac J = Jacobian(x, y, dr, ds);
            // [dxi;deta] = J^-T [-fx;-fy]; J^-T: x = xr*dxi + xs*deta
            double det = J.xr * J.ys - J.xs * J.yr;
            double dxi = (-fx * J.ys + fy * J.xs) / det;
            double deta = (-fy * J.xr + fx * J.yr) / det;
            xi += dxi; eta += deta;
            if (std::fabs(dxi) + std::fabs(deta) < 1e-12) break;
         }
         double out = std::max(std::fabs(xi), std::fabs(eta));
         if (out <= 1 + 1e-9) { best.shell = e; best.xi = std::clamp(xi, -1.0, 1.0); best.eta = std::clamp(eta, -1.0, 1.0); return best; }
         if (out < bestOut && out < 1 + 1e-6) { bestOut = out; best.shell = e; best.xi = std::clamp(xi, -1.0, 1.0); best.eta = std::clamp(eta, -1.0, 1.0); }
      }
      return best;
   }

   void Model::ShellShape(const PointLoc& loc, double N[4]) const
   {
      double dr[4], ds[4];
      Shape(loc.xi, loc.eta, N, dr, ds);
   }

   double Model::Interpolate(const std::vector<double>& nodal, const PointLoc& loc) const
   {
      if (loc.shell < 0) return 0.0;
      double N[4]; ShellShape(loc, N);
      double v = 0;
      for (int k = 0; k < 4; k++) v += N[k] * nodal[m_Shells[loc.shell].n[k]];
      return v;
   }

   double Model::Interpolate(const std::vector<double>& nodal, double x, double y) const
   {
      return Interpolate(nodal, Locate(x, y));
   }
}
