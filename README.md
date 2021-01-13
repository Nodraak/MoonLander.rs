# README.md

(Moon) lander.

Diagram: https://blog.nodraak.fr/images/2020/08/aerospace-sim/Architecture.svg (From https://blog.nodraak.fr/2020/08/aerospace-sim-1-preliminary-study/)

GNC is implemented in Rust.

I/O will be implemented via adapters:

* Custom simulator -> TCP server (Rust). TCP connection will be JSON (en|de)coded with Serde.
* KSP -> embdded Python using Krpc lib (protobuf to KSP) (https://github.com/PyO3/pyo3 + https://krpc.github.io/krpc/).

KSP lib options for altitude input (used by Nav):

* Kerbal Engineer Redux displays "Altitude to Terrain"
* Radar Altimeter inside most (if not all) IVA
* KER or mechjeb can give you that info, mechjeb can also give your time to impact.
* KerbNet
