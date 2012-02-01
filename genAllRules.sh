
rulefolder=~/cfgProject/cfg3d/rules
cd $rulefolder
rm  rule_*

set -e

cd /home/aa755/cfgProject/data/gold/realData/CPU/gold/featureLearning
./genCPUFeats.sh
#/home/aa755/cfgProject/data/gold/realData/CPU/gold/featureLearning/genCPUFeats.sh
cp rule_* $rulefolder/


cd /home/aa755/cfgProject/data/gold/realData/monitorTripod/final
./genMonitorFeats.sh
#/home/aa755/cfgProject/data/gold/realData/monitorTripod/final/genMonitorFeats.sh
cp rule_* $rulefolder/



cd /home/aa755/cfgProject/data/localBackup/allPCDs/wallPCDs
./learn.sh
#/home/aa755/cfgProject/data/localBackup/allPCDs/wallPCDs/learn.sh
cp rule_* $rulefolder/


cd /home/aa755/cfgProject/data/gold/Sketchups/
#/home/aa755/cfgProject/data/gold/Sketchups/genPrinterFeats.sh
./genPrinterFeats.sh
cp rule_* $rulefolder/

#get the distributions 
cd $rulefolder
nohup matlab -nodesktop -nosplash -r genAllRulesDistScript
# manual rules
echo "0.9305,0.2428,0.6000,1.00" > rule_5Floor__5Plane.out
echo "0.0021,0.0663,-0.100,0.1" >> rule_5Floor__5Plane.out
echo "0.05,0.1,0.0000,0.700" >> rule_4Wall__5Plane.out

for file in `ls rule_*printer*__5Plane.out`
do
    ./removeColorFeats.sh $file > temp
    cat printer.color > $file
    cat temp >> $file
done
