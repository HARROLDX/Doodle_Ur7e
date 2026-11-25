# Attribution

This project adapts the drawing workflow of **DoodleDroid**, by David Matthews,
Christian Tongate, Harrison Bounds, Yanni Kechriotis and Zhengxiao Han:
https://github.com/HarrisonBounds/DoodleDroid
Pinned reference: `1732b5be7e7e4ff19ab84d9195e61e29bc6a1eff`.
The upstream WTFPL text is retained in `DoodleDroid.LICENSE`.

`image.py`'s `getdots` and `connectdots` are adapted from the upstream linedraw
module, derived from **Lingdong Huang's linedraw** (2017):
https://github.com/LingDong-/linedraw
Its MIT license is retained in `linedraw.LICENSE`. Changes include safe contour
filtering, deterministic simplification, no random perturbations, aspect-preserving
normalization, and no interactive/file-system side effects in the extractor.

New integration code is MIT-licensed (root LICENSE). No upstream example portraits,
third-party photos, coursework solutions, PDF handouts, or pretrained assets are redistributed.
