./test.sh --gap-us 250 --mode joint --joint 2 --run G0A-J2-1
./test.sh --gap-us 0   --mode joint --joint 6 --run G0B-b0-1
./test.sh --gap-us 250 --mode joint --joint 6 --kp 12.5 --run G0C-lo-1
./test.sh --gap-us 250 --mode joint --joint 6 --run G0A-J6-2
./test.sh --gap-us 250 --mode joint --joint 2 --run G0A-J2-2
./test.sh --gap-us 0   --mode joint --joint 6 --run G0B-b0-2
./test.sh --gap-us 250 --mode joint --joint 6 --kp 12.5 --run G0C-lo-2
./test.sh --gap-us 250 --mode joint --joint 6 --run G0A-J6-3
./test.sh --gap-us 250 --mode joint --joint 2 --run G0A-J2-3
./test.sh --gap-us 0   --mode joint --joint 6 --run G0B-b0-3
./test.sh --gap-us 250 --mode joint --joint 6 --kp 12.5 --run G0C-lo-3
./test.sh --gap-us 250 --mode joint --joint 6 --run G0A-J6-4
./test.sh --gap-us 250 --mode joint --joint 2 --run G0A-J2-4
./test.sh --gap-us 250 --mode joint --joint 6 --run G0A-J6-5
./test.sh --gap-us 250 --mode joint --joint 2 --run G0A-J2-5
# —— P2 可选（时间富余再跑）——
./test.sh --gap-us 250 --mode joint --joint 6 --period-triangle 2 --run G0D-p2
./test.sh --gap-us 250 --mode joint --joint 6 --period-triangle 8 --run G0D-p8