# UAV Optimal Path Planning in Radar Threat Environments

Bu çalışma, İnsansız Hava Araçlarının (İHA) görev icra ederken hava savunma radarları ve tehdit alanlarından kaçınarak hedef noktaya minimum risk ve enerji ile ulaşmasını sağlayan $A^*$ tabanlı bir rota optimizasyon sistemidir.

## Problem ve Matematiksel Modelleme
- **Maliyet Haritası (Cost Surface):** Radarların etki alanı, merkezden dışa doğru azalan 2 boyutlu Gauss dağılımı ile cezalandırılmıştır:
  $$C(x, y) = 1.0 + \sum I_k \cdot \exp\left(-\frac{d_k^2}{2\sigma_k^2}\right)$$
- **Arama Algoritması ($A^*$):** Rotanın toplam tahmini maliyeti $f(n) = g(n) + h(n)$ formülü ile hesaplanır. Burada $g(n)$ başlangıçtan itibaren harcanan gerçek enerji/tehdit maliyetini, $h(n)$ ise hedefe olan Öklid uzaklığı sezgiseli (admissible heuristic) değerini temsil eder.

## Kurulum ve Çalıştırma
```bash
pip install numpy matplotlib
python path_planner.py
