Qubit demonstration open day CWI 2019
=====================================

Welcome to the *flaming raging orb of quantum supremacy*.

`Orb`
-----

The folder `orb` is a visual code project that contains all the
code that is to be run on the orb itself. Open this folder in visual
code and make sure that you install the platformIO plugin in order
to successfully compile the program and send it to the orb.

`Web`
-----

The folder `web` is a simple webserver that handles the
communication with the orb and contains some exercises.

In order to run it, first connect the orb to your computer via USB.
Next, check with `dmesg` to which port it is connected, typically
`/dev/ttyUSB0`. If it is connected to a different port, then you
can change the address in `app.py`. Next, make sure you have
`pyserial` and `flask` installed, and run with

``python3 app.py``

Now you can connect to the webserver by navigating to
`localhost:5001` in a browser.
