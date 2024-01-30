# Warn the user that the script will remove local changes
# prompt to continue or abort
$msg = "This will remove your local changes to the project. Continue? [y/n]"
$response = Read-Host -Prompt $msg
if ($response -ne 'y') {
   echo "Aborting"
   exit
}

git reset --hard
echo "ESP hosted: initialize submodule esp-idf"
git submodule update --init --depth=1
echo "ESP hosted: initialize submodule for esp-idf"
cd esp-idf
git submodule update --init --depth=1
echo "ESP hosted: installing prerequisites for esp-idf"
.\install.ps1
cd ..
echo "###### Setup Done ######"
