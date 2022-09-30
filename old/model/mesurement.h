#pragma once
struct Measurement {
  Measurement(int a, int b, int k) : a(a), b(b), k(k) {}
  // measured, beam, index of
  int a, b, k;
};

using Measurements = std::vector<Measurement>;