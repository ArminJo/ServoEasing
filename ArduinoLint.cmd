if exist keywords.txt  (
  E:\Elektro\arduino_common\arduino-lint.exe --library-manager update --compliance strict
) else (
  echo "Please change directory to library root"
)
pause