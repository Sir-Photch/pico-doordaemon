#!/usr/bin/fish

while true

    set -l msg $(string split ' ' (string trim (ncat -l 1337)))
    echo $msg
    if test "$msg[1]" = "ALERT"
        # mpg123 -q "$(pwd)/daemon.mp3" &
        notify-send --transient -a "doordaemon" -i "$(pwd)/daemon.png" "Es kommen Leute zur Burg!" "Distanz: $msg[2] mm"
    end

end