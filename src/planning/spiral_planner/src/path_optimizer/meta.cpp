#include "path_optimizer/meta.h"

std::vector<int> sort_indexes(const std::vector<std::pair<int, double>> &v) {
  std::vector<int> idx(v.size());
  for (size_t i = 0; i != idx.size(); ++i)
    idx[i] = v[i].first;
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) { return v[i1].second < v[i2].second; });
  return idx;
}
