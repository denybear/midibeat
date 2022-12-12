# midibeat
Raspberry Pico sends MIDI clocks to a groovebox at each press of a button

The problem:

When playing with a groovebox in a band, and in particular with a drummer, it is very difficult to adapt groovebox rhythm to the one of the band.
Indeed, groovebax provides a very straight rhythm, while a drummer is not that precise. After a couple of minutes, groovebox and band end up in total desynchronisation. 

The solution:

midibeat is a raspberry pico software that allows groovebox to play in sync with a band.
By press of a button (switch) at every beat, pico sends midi-clock information to the groovebox allowing it to stay in sync with the beat set by the drummer (which could be fluctuating).

In our band, the switch is managed by the singer.
midibeat is very useful in small bands such as duo or trio, where you want the groovebox to provide another layer of drums or harmony in sync with the rest of the band.

For this to work, your groovebox shall accept midi-clock IN information; it works well on my Novation Circuit for instance.
