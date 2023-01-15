#!/usr/bin/bash
#
# Usage:
#    bash setup_caen.sh <uniqname>
#
# Example:
#    bash setup_caen.sh anandan
#
#
. virtualenv_config.sh

# Used by `_virtualenv` to select the correct virtualenv invocation. 0 if we
# should invoke virtualenv as `python -m virtualenv`, or 1 if we should invoke
# it as `virtualenv`. This is set by `install_virtualenv`, so run it before
# running `_virtualenv`.
virtualenv_is_python_module=

# Run the appropriate virtualenv command.
_virtualenv() {
    case "$virtualenv_is_python_module" in
        0)
            virtualenv "$@"
            ;;
        1)
            python -m virtualenv "$@"
            ;;
        *)
            echo "Didn't know how to execute virtualenv."
            exit 1
    esac
}

# Install the `virtualenv` package if it's not already installed.
install_virtualenv() {
    if command -v virtualenv >/dev/null 2>&1; then
        virtualenv_is_python_module=0
        echo 'virtualenv was already installed'
    elif python -m virtualenv >/dev/null 2>&1; then
        virtualenv_is_python_module=1
        echo 'virtualenv was already installed'
    else
        # Install virtualenv on CAEN. See also
        # https://github.com/agiantwhale/caenhacks#install-python-packages-globally-with-pip
        echo 'Installing virtualenv...'
        mkdir -p ~/.local/lib/python2.7/site-packages
        easy_install --prefix=~/.local virtualenv

        virtualenv_is_python_module=1
        echo 'Successfully installed virtualenv'
    fi
}

setup_virtualenv() {
    echo 'Setting up a virtualenv for the visualizer script'
    echo "Location: $VIRTUALENV_DIR"

    _virtualenv -p python2.7 "$VIRTUALENV_DIR"

    . "$VIRTUALENV_DIR"/bin/activate

    echo 'Installing dependencies... (this may take a while)'
    pip install networkx
    pip install requests
    pip install matplotlib
    pip install configobj
    pip install mpltools
}

setup_webpage() {
    echo 'Setting up your personal webpage on University of Michigan servers...'
    echo "$USER" | ssh "$USER"@login.itd.umich.edu -t -t ~umweb/bin/make-webspace
}

main() {
    install_virtualenv
    setup_virtualenv
    setup_webpage
}

main
