while IFS=$'\n' read -r line
do
    printf "%-${COLUMNS}s\n" "$line"
    #printf "$line\n"
    sleep 0.016667
done < vive_tilt.txt
