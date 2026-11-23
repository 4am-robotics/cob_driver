You can use TTS speach output by using the
[cob_script_server](https://index.ros.org/p/cob_script_server).

## Starting Sound

NOTE: If you login remotely via ssh, login without -X.


```
roslaunch cob_sound sound.launch
```



## Usage

You can either call and action or a service, e.g.


```
rosservice call /sound/say "Hello, my name is Care-O-bot."
```



You can mute and unmute the output 

```
rosservice call /sound/mute
rosservice call /sound/unmute
```



### Using festival

By default [festival](http://www.cstr.ed.ac.uk/projects/festival) is
used.

### Using cepstral

For [cepstral](http://www.cepstral.com) you need to buy a license key.

For using cepstral instead of festival please set the mode to cepstral
with 

```
rosparam set /sound/mode cepstral
```



#### Installing cepstral vices and license

Download and install voice. Adopted from
[this tutorial](http://fonality.com/trixbox/forums/trixbox-forums/open-discussion/mini-howto-install-cepstral-asterisk-weather-station-zip-nerdv).

- Download a voice file from
  <http://www.cepstral.com/downloads/personal.php> e.g. Linux 32bit or
  64bit, US English, David: 

```
wget http://www.cepstral.com/downloads/installers/linux32/Cepstral_David_i386-linux_6.2.3.873.tar.gz
wget http://www.cepstral.com/downloads/installers/linux64/Cepstral_David_x86-64-linux_6.2.3.873.tar.gz
```

 e.g. Linux 32bit or
  64bit, US English, Diane: 

```
wget http://www.cepstral.com/downloads/installers/linux32/Cepstral_Diane_i386-linux_6.2.3.873.tar.gz
wget http://www.cepstral.com/downloads/installers/linux64/Cepstral_Diane_x86-64-linux_6.2.3.873.gz
```

 e.g. Linux 32bit or
  64bit, German, Matthias: 

```
wget http://www.cepstral.com/downloads/installers/linux32/Cepstral_Matthias_i386-linux_6.2.3.873.tar.gz
wget http://www.cepstral.com/downloads/installers/linux64/Cepstral_Matthias_x86-64-linux_6.2.3.873.tar.gz
```

 e.g. Linux 32bit or
  64bit, German, Katrin: 

```
wget http://www.cepstral.com/downloads/installers/linux32/Cepstral_Katrin_i386-linux_6.2.3.873.tar.gz
wget http://www.cepstral.com/downloads/installers/linux64/Cepstral_Katrin_x86-64-linux_6.2.3.873.tar.gz
```



<!-- -->

- unpack and install 

```
tar xzvf Cepstral_David_x86-64-linux_6.2.3.873.tar.gz
cd Cepstral_David_x86-64-linux_6.2.3.873
sudo ./install.sh
sudo touch /etc/ld.so.conf.d/cepstral.conf
sudo sh -c 'echo "/opt/swift/lib" > /etc/ld.so.conf.d/cepstral.conf'
sudo ldconfig
```



<!-- -->

- apply fix for Ubuntu > 12.04 

```
roscd cob_sound
sudo ./fix_swift_for_precise.sh
```



if that fix doesn't help, try to install alsa-oss


```
sudo apt-get install alsa-oss
```

 and modify /usr/local/bin/swift_fix to contain


```
if test -x /usr/bin/padsp ; then  exec aoss /opt/swift/bin/swift "$@" ; else  exec /opt/swift/bin/swift "$@" ; fi
```



- Register voice with license (you need to buy a license from
  <http://www.cepstral.com>, for IPA robots ask UHR or FMW for license
  key) 

```
sudo swift --reg-voice
```



## Setting the sound volume

Either use alsamixer: 

```
alsamixer
```



or use alsactl: For setting the sound to xx% use


```
amixer -c 1 set Speaker xx%
```

 or 

```
amixer -c 1 set PCM xx%
```

 5% is a
reasonable volume. To store the settings permanently, type


```
sudo alsactl store
```
