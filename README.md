Skandal3.0
==========

Skandal is a 3D Laser Scanner to create STL file from a small real object
and print it on a 3D Printer.

## Usage
In the Skandal3.0-master directory, open a terminal

 ./scan

 or double clic on scan, Run in Terminal

Press Esc key in active window to quit and get the terminal.

## Unfinished
You must set the device video number in skandal.ini:

video = 0
or
video = 1

## Documentation
On the Labomedia wiki
http://wiki.labomedia.org/index.php/Cat%C3%A9gorie:Skandal


## Installation
Tested on Linux Mint 15

See:

http://wiki.labomedia.org/index.php/Laser_Scanner_3D_SkanDal_Logiciel#Installation_pour_python_3_et_OpenCV_3.0


## Running the Tests
With a webcam and without Arduino Card


## Requirements
* python3

* OpenCV 3.0


## Contributing
Write on the discussion page in the wiki:
http://wiki.labomedia.org/index.php/Discussion:Laser_Scanner_3D_SkanDal


## Credits
Merci à:

* Jeremy avec qui je me suis lancé dans cette aventure

* Olivier pour son enthousiasme

* Labomedia

* Alexandre pour ses conseils techniques de programmation

* Romain qui nous a appris qu'il fallait faire un Readme plein de poésie

Je continuerai la liste si je reçois un Oscar python dans la catégorie Vétéran.


## License
Skandal is released under the GENERAL PUBLIC LICENSE Version 2, June 1991.
See the bundled LICENSE file for details.


## Bugs:
* Often, OpenCV Windows doesn't close with cv2.destroyAllWindows()
    so a small image is open to close the cam windows
