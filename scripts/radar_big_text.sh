#!/bin/bash

watch -n 0.5 'figlet -f mono12 $(tail -1 $(ls -tr *.tsv | tail -1) | awk "{print \$9}")'
