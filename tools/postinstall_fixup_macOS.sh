#!/bin/bash

INSTALL_DIR="$1"

echo "Fixing up libs manually ..."

check-all-deps() {
    mode="$1"
    pass="$2"

    (
    find "$INSTALL_DIR" -type f -name "Qt*" -path "*/Qt*.framework/Versions/*/Qt*" | grep -v "Headers"
    find "$INSTALL_DIR" -type f -name "*.dylib"
    find "$INSTALL_DIR" -type f -name "*.so"
    find "$INSTALL_DIR" -type f -name "runSofa*" -path "*/bin/*"
    ) | while read lib; do
        echo "  Checking (pass $pass) $lib"

        libqt=""
        libboost=""
        libicu=""
        libglew=""
        libjpeg=""
        libpng=""
        libtiff=""
        dependencies="$( otool -L $lib | tail -n +2 | perl -p -e 's/^[\t ]+(.*) \(.*$/\1/g' )"

        is_fixup_needed="false"
        if echo "$dependencies" | grep --quiet "/Qt"       ||
           echo "$dependencies" | grep --quiet "/libboost" ||
           echo "$dependencies" | grep --quiet "/libicu"   ||
           echo "$dependencies" | grep --quiet "/libGLEW"  ||
           echo "$dependencies" | grep --quiet "/libjpeg"  ||
           echo "$dependencies" | grep --quiet "/libpng"   ; then
            is_fixup_needed="true"
        fi
        if [[ "$is_fixup_needed" == "false" ]]; then
            continue # skip this lib
        fi

        (echo "$dependencies") | while read dep; do
            if libqt="$(echo $dep | egrep -o "/Qt[A-Za-z]*$" | cut -c2-)" && [ -n "$libqt" ]; then
                libname="$libqt"
            elif libboost="$(echo $dep | egrep -o "/libboost_[^\/]*?\.dylib" | cut -c2-)" && [ -n "$libboost" ]; then
                libname="$libboost"
            elif libicu="$(echo $dep | egrep -o "/libicu[^\/]*?\.dylib$" | cut -c2-)" && [ -n "$libicu" ]; then
                libname="$libicu"
            elif libglew="$(echo $dep | egrep -o "/libGLEW[^\/]*?\.dylib$" | cut -c2-)" && [ -n "$libglew" ]; then
                libname="$libglew"
            elif libjpeg="$(echo $dep | egrep -o "/libjpeg[^\/]*?\.dylib$" | cut -c2-)" && [ -n "$libjpeg" ]; then
                libname="$libjpeg"
            elif libpng="$(echo $dep | egrep -o "/libpng[^\/]*?\.dylib$" | cut -c2-)" && [ -n "$libpng" ]; then
                libname="$libpng"
            elif libtiff="$(echo $dep | egrep -o "/libtiff[^\/]*?\.dylib$" | cut -c2-)" && [ -n "$libtiff" ]; then
                libname="$libtiff"
            else
                if [[ "$dep" == "/usr/local/"* ]]; then
                    echo "WARNING: no fixup rule set for: $lib"
                fi
                # this dep is not a lib to fixup
                continue
            fi

            if [[ "$mode" == "copy" ]]; then
                if [ -n "$libqt" ] && [ -d "$QT_LIB_DIR" ]; then
                    originlib="$QT_LIB_DIR/$libqt.framework"
                    destlib="$INSTALL_DIR/lib/$libqt.framework"
                else
                    originlib="$dep"
                    destlib="$INSTALL_DIR/lib/$libname"
                fi
                if [ -e $originlib ] && [ ! -e $destlib ]; then
                    echo "    cp -Rf $dep $INSTALL_DIR/lib"
                    cp -Rf $originlib $INSTALL_DIR/lib
                fi
            elif [[ "$mode" == "fixup" ]]; then
                if [ -n "$libqt" ]; then
                    rpathlib="$libqt.framework/$libqt"
                else
                    rpathlib="$libname"
                fi
                libbasename="$(basename $lib)"
                echo "install_name_tool -change $dep @rpath/$rpathlib $libbasename"
                install_name_tool -change $dep @rpath/$rpathlib $lib
            fi
        done
    done
}

check-all-deps "copy" "1/4"
check-all-deps "copy" "2/4"
check-all-deps "copy" "3/4"
chmod -R 755 $INSTALL_DIR/lib
check-all-deps "fixup" "4/4"

echo "Done."
