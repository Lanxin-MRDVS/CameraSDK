
#!/bin/bash
URRENT_PWD=$(cd "$(dirname "$0")";pwd)
 chmod 777 ./*
 export LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH

 ./middleware empty


exit 0

