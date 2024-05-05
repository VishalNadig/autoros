#! /usr/bin/bash
autoros() {
    # if ["$1" = 'hello']; then
    #     touch config.yaml
    #     echo 'Created config.yaml file in $HOME directory!';

    # elif [ "$1" = 'create_workspace' ]; then
    #     echo 'Bye, World!';

    # elif [ "$1" = 'create_package' ]; then
    #     echo 'Bye, World!';

    # elif [ "$1" = 'create_config_template' ]; then
    #     echo 'Bye, World!';

    # elif [ "$1" = 'initialize' ]; then
    #     echo 'Bye, World!';

    # # else [ "$1" = 'hello' ];
    # #     echo 'Hello, World!';
    # fi
    read command
    echo $command
}