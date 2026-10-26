///////////////////////////////////////////////////////////////////////
// RfaFem - plate + eccentric beam finite element engine for refined
// live load analysis (AASHTO LRFD 4.6.3).
//
// Self-contained (standard library only). Consistent units are up to the
// caller; the PGSuper agent uses SI base units (m, N, Pa).
//
// Model idealisation
//   * The deck is a flat plate in the global XY plane at z = 0 (deck
//     mid-surface). Each node has 6 DOF: u, v, w, rx, ry, rz.
//   * Deck elements: 4-node flat shells = membrane (Q4 + incompatible
//     modes, Taylor correction) + bending (MITC4 Mindlin plate). The
//     drilling DOF gets a small artificial stiffness.
//   * Girders: 3D Euler-Bernoulli beams whose axis lies a distance `ecc`
//     below the deck node (rigid link, eliminated by transformation, so
//     beams are attached directly to deck nodes). Composite action comes
//     from the model rather than from a transformed section.
//   * Supports: exact elimination of vertical w (a vertical support at any
//     depth below the node constrains w exactly) plus elastic springs in
//     any direction at an offset point (bearings, horizontal restraint).
//
// Responses are linear functionals c.u of the displacements. Influence
// surfaces come from one adjoint solve per response: the effect of a unit
// downward load at node j is -(K^-1 c)_wj (K is symmetric).
///////////////////////////////////////////////////////////////////////
#pragma once

#include <vector>
#include <array>
#include <map>
#include <string>
#include <cmath>

namespace rfa
{
   struct Vec3
   {
      double x = 0, y = 0, z = 0;
      Vec3() = default;
      Vec3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
      Vec3 operator+(const Vec3& o) const { return { x + o.x,y + o.y,z + o.z }; }
      Vec3 operator-(const Vec3& o) const { return { x - o.x,y - o.y,z - o.z }; }
      Vec3 operator*(double s) const { return { x * s,y * s,z * s }; }
      double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
      Vec3 cross(const Vec3& o) const { return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x }; }
      double norm() const { return std::sqrt(dot(*this)); }
      Vec3 unit() const { double n = norm(); return n > 0 ? (*this) * (1.0 / n) : *this; }
   };

   enum Dof { U = 0, V = 1, W = 2, RX = 3, RY = 4, RZ = 5 };

   struct ShellProps { double E = 0, nu = 0.2, t = 0; };

   // Beam section. Strong-axis bending is about the local horizontal axis
   // (vertical bending of the girder).
   struct BeamProps
   {
      double E = 0, G = 0;
      double A = 0;
      double Iv = 0;   // moment of inertia for vertical bending (strong axis)
      double Il = 0;   // moment of inertia for lateral bending (weak axis)
      double J = 0;    // St. Venant torsion constant
   };

   struct Shell { std::array<int, 4> n{}; int props = 0; };

   struct Beam
   {
      std::array<int, 2> n{};
      int props = 0;
      std::array<double, 2> ecc{ 0,0 }; // distance from deck node down to beam centroid at each end
   };

   // Elastic spring to ground at point (node + offset) along unit direction dir.
   struct Spring { int node = 0; Vec3 dir; Vec3 offset; double k = 0; bool rotational = false; };

   // Sparse linear functional over global (node,dof) pairs
   struct Functional
   {
      std::map<int, double> c;      // global dof index (6*node+dof) -> coefficient
      std::map<int, double> direct; // fixed w dof -> value when a unit downward load sits right on it
      void add(int gdof, double v) { c[gdof] += v; }
      void add(const Functional& o, double s = 1.0)
      {
         for (auto& kv : o.c) c[kv.first] += s * kv.second;
         for (auto& kv : o.direct) direct[kv.first] += s * kv.second;
      }
   };

   // Location of a plan point inside the mesh
   struct PointLoc { int shell = -1; double xi = 0, eta = 0; };

   class SkylineSolver; // fwd

   class Model
   {
   public:
      Model();
      ~Model();
      Model(const Model&) = delete;
      Model& operator=(const Model&) = delete;

      // ---- definition ----
      int AddNode(double x, double y);
      int AddShellProps(const ShellProps& p);
      int AddBeamProps(const BeamProps& p);
      int AddShell(int n1, int n2, int n3, int n4, int props); // counter-clockwise in plan
      int AddBeam(int n1, int n2, int props, double ecc1, double ecc2);
      void FixDof(int node, int dof);                         // exact elimination
      int AddSpring(int node, const Vec3& dir, const Vec3& offset, double k);
      int AddRotationalSpring(int node, const Vec3& axis, double k);   // moment spring about axis
      void SetDrillingFactor(double f) { m_DrillFactor = f; }

      size_t NumNodes() const { return m_Nodes.size(); }
      size_t NumShells() const { return m_Shells.size(); }
      size_t NumBeams() const { return m_Beams.size(); }
      const Vec3& NodePos(int n) const { return m_Nodes[n]; }
      const Shell& GetShell(int i) const { return m_Shells[i]; }
      const Beam& GetBeam(int i) const { return m_Beams[i]; }
      bool IsFixed(int node, int dof) const { return m_Fixed[6 * node + dof] != 0; }
      int NumEquations() const { return m_NumEq; }

      // ---- analysis ----
      // Assemble and factor. Returns false (and sets error) if singular.
      bool Factor(std::string* error = nullptr);
      // Solve K u = f, where f is a full global vector (6*nnodes). Returns
      // full displacement vector (constrained dofs = 0).
      std::vector<double> Solve(const std::vector<double>& f) const;
      // Static solution for a single vertical point load (downward positive P) at a plan point
      std::vector<double> SolvePointLoad(double x, double y, double P) const;

      // ---- response functionals ----
      // Beam end forces in local axes (x' along beam start->end, z' up-ish, y' = z' x x')
      // comp: 0..5 = Fx,Fy,Fz,Mx,My,Mz acting ON the element at end (0 = start, 1 = end).
      Functional BeamEndForce(int beam, int end, int comp) const;
      // Internal actions at a beam end section using girder sign conventions:
      //   N  axial (tension +), V vertical shear on the face, Msag sagging moment (+)
      Functional BeamAxial(int beam, int end) const;
      Functional BeamShear(int beam, int end) const;
      Functional BeamMomentSag(int beam, int end) const;
      // Upward reaction at a fixed w dof
      Functional Reaction(int node, int dof) const;
      // Upward reaction of a spring
      Functional SpringForce(int spring) const;
      // Plate stress resultants at a point inside a shell element (per unit width),
      // tensor components resolved along unit direction t (in plan):
      //   Mtt (Mindlin sign: + = tension at top), Ntt (tension +), Qt (shear on face normal t)
      Functional PlateMtt(int shell, double xi, double eta, const Vec3& t) const;
      Functional PlateNtt(int shell, double xi, double eta, const Vec3& t) const;
      Functional PlateQt(int shell, double xi, double eta, const Vec3& t) const;
      // Element nodal force (global axes) that node `localNode` of the element exerts ON the
      // element, component dof (0..5): one row of the element stiffness in global dofs.
      // Used for free-body section resultants (exact equilibrium).
      Functional ShellNodalForce(int shell, int localNode, int dof) const;
      Functional BeamNodalForce(int beam, int localNode, int dof) const;
      // Displacement functional (for tests)
      Functional Disp(int node, int dof) const { Functional f; f.add(6 * node + dof, 1.0); return f; }

      double Evaluate(const Functional& f, const std::vector<double>& u) const;

      // Influence surface: value of functional f due to a unit DOWNWARD load at each node
      std::vector<double> InfluenceSurface(const Functional& f) const;
      // Many at once (one adjoint solve each)
      std::vector<std::vector<double>> InfluenceSurfaces(const std::vector<Functional>& f) const;

      // ---- geometry helpers ----
      // Locate a plan point (bucket search + Newton inverse map). shell=-1 if outside.
      PointLoc Locate(double x, double y) const;
      // Interpolate nodal field at a point (bilinear). Returns 0 outside the mesh.
      double Interpolate(const std::vector<double>& nodal, double x, double y) const;
      double Interpolate(const std::vector<double>& nodal, const PointLoc& loc) const;
      void ShellShape(const PointLoc& loc, double N[4]) const;

   private:
      std::vector<Vec3> m_Nodes;
      std::vector<ShellProps> m_ShellProps;
      std::vector<BeamProps> m_BeamProps;
      std::vector<Shell> m_Shells;
      std::vector<Beam> m_Beams;
      std::vector<Spring> m_Springs;
      std::vector<char> m_Fixed;      // per global dof
      std::vector<int> m_EqNum;       // global dof -> equation (-1 = fixed)
      int m_NumEq = 0;
      double m_DrillFactor = 1.0e-3;
      SkylineSolver* m_pSolver = nullptr;
      std::map<int, std::map<int, double>> m_ReactionRows; // fixed global dof -> (global dof -> K)

      // bucket grid for Locate
      double m_BX0 = 0, m_BY0 = 0, m_BDX = 1, m_BDY = 1; int m_BNX = 0, m_BNY = 0;
      std::vector<std::vector<int>> m_Buckets;
      void BuildBuckets() const;
      mutable bool m_BucketsBuilt = false;

      // element matrices
      void ShellStiffness(int e, double K[24][24]) const;
      void BeamLocalStiffness(int e, double k[12][12], double L) const;
      void BeamTransform(int e, double T[12][12], double& L) const; // local beam dofs = T * master dofs
      void ShellMembraneRecovery(int e, double xi, double eta, double B[3][8]) const;
      void ShellBendingRecovery(int e, double xi, double eta, double Bb[3][12], double Bs[2][12]) const;

      void NumberEquations();
   };
}
