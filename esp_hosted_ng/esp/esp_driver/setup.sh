git reset --hard
echo "ESP hosted: initialize submodule esp-idf"
git submodule update --init --depth=1
echo "ESP hosted: initialize submodule for esp-idf"
cd esp-idf && git submodule update --init --depth=1
echo "ESP hosted: installing prerequisites for esp-idf"
./install.sh
cd ..
echo "ESP hosted: replacing wireless libraries"
rm -r esp-idf/components/esp_wifi/lib/*
cp -r lib/* esp-idf/components/esp_wifi/lib/
echo "###### Setup Done ######"
