# WaveRave: Software Defined Radio but [we're crabs at a rave][crabwave]

WaveRave is (eventually) a collection of Rust crates for doing software defined 
radio stuff in an asychronous way. Right now it's just a [HackRF][hackrf] 
backend crate, but maybe I'll write more!

- [`waverave-hackrf`][waverave-hackrf]: HackRF radio backend, implemented with 
	[`nusb`][nusb].
- [`waverave-hackrf-bin`][waverave-hackrf-bin]: HackRF binary utilities built 
 	on [`waverave-hackrf`]

[crabrave]: https://www.youtube.com/watch?v=cE0wfjsybIQ
[hackrf]: https://greatscottgadgets.com/hackrf/one/
[nusb]: https://github.com/kevinmehall/nusb
[waverave-hackrf]: ./waverave-hackrf
[waverave-hackrf-bin]: ./waverave-hackrf-bin
