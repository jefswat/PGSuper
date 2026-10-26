// Mesh convergence of refined LLDFs (synthetic Type IV bridge from test_bridge.cpp)
#define main test_bridge_main
#include "test_bridge.cpp"
#undef main
int main()
{
   double sizes[3][2] = { {0.90, 1.50}, {0.45, 0.75}, {0.225, 0.375} };
   for (int c = 0; c < 2; c++)
   {
      Spec sp; if (c == 1) { sp.skewDeg = 30; }
      std::printf("\n%s\n  mesh (m)        nodes   gM+ A   gM+ B   gM+ C   gV A    gV C    secs\n", c ? "30 deg skew" : "square");
      for (auto& s : sizes)
      {
         BridgeInput b = MakeBridge(sp);
         AnalysisOptions opt; opt.keepSurfaces = false;
         opt.mesh.transverseSize = s[0]; opt.mesh.longitudinalSize = s[1];
         Results r = RunRefinedAnalysis(b, opt);
         if (!r.ok) { std::printf("failed %s\n", r.error.c_str()); continue; }
         auto G = [&](int g) { for (auto& x : r.girders) if (x.girder == g) return x; return r.girders[0]; };
         std::printf("  %.3f x %.3f  %7d   %.4f  %.4f  %.4f  %.4f  %.4f  %5.1f\n", s[0], s[1], r.nodes, G(0).gMpos, G(1).gMpos, G(2).gMpos, G(0).gV, G(2).gV, r.seconds);
      }
   }
   return 0;
}
