#!/bin/bash
export PATH=$PATH:/home/linuxbrew/.linuxbrew/bin/
export HOMEBREW_NO_INSTALL_FROM_API=1
directory=$(cat ${CMAKE_CURRENT_SOURCE_DIR}/.git/config | grep origin -A 1 | head -n 2 | tail -n 1 | cut -d ':' -f 2)
repository=$(echo $directory | cut -d '/' -f 1)
project=$(echo $directory | cut -d '/' -f 2)
brew tap --force homebrew/core
brew create --cmake  'https://github.com/ahp-electronics/libahp-gt/archive/refs/tags/v@AHPGT_VERSION@.tar.gz' --set-name '$project@@AHPGT_VERSION@' --set-version '@AHPGT_VERSION@' --tap $repository/homebrew-$project

