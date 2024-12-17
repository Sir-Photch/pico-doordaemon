#!/usr/bin/fish

set -l basedir "$(dirname (readlink -f (status filename)))"

while true

    set -l msg $(string split ' ' (string trim (ncat -l 1337)))
    echo $msg
    if test "$msg[1]" = "ALERT"
	mpg123 -q "$basedir/daemon.mp3" &
        notify-send --transient -a "doordaemon" -i "$basedir/daemon.png" "Es kommen Leute zur Burg!" "Distanz: $msg[2] mm"
    	sleep 30
    end
end
