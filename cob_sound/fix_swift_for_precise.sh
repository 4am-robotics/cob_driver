#!/bin/bash

sudo touch -f /usr/local/bin/swift_fix
sudo echo "#!/bin/bash" > /usr/local/bin/swift_fix
sudo echo 'if test -x /usr/bin/padsp ; then  exec /usr/bin/padsp /opt/swift/bin/swift "$@" ; else  exec /opt/swift/bin/swift "$@" ; fi' >> /usr/local/bin/swift_fix
sudo chmod +x /usr/local/bin/swift_fix
sudo ln -sf /usr/local/bin/swift_fix /usr/local/bin/swift
