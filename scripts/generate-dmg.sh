#!/usr/bin/env bash

# This script requires create-dmg to be installed from https://github.com/sindresorhus/create-dmg
BUILD_CONFIG=$1

fail()
{
	echo "$1" 1>&2
	exit 1
}

if [ "$BUILD_CONFIG" != "Debug" ] && [ "$BUILD_CONFIG" != "Release" ]; then
  fail "Invalid build configuration - expected 'Debug' or 'Release'"
fi

BUILD_ROOT=$PWD/build
SOURCE_ROOT=$PWD
BUILD_FOLDER=$BUILD_ROOT/build-$BUILD_CONFIG
INSTALLER_FOLDER=$BUILD_ROOT/installer-$BUILD_CONFIG
APP_BUNDLE="$BUILD_FOLDER/app/Moonlight-Metal.app"

if [ -n "$CI_VERSION" ]; then
  VERSION=$CI_VERSION
else
  VERSION=`cat $SOURCE_ROOT/app/version.txt`
fi

if [ "$SIGNING_PROVIDER_SHORTNAME" == "" ]; then
  SIGNING_PROVIDER_SHORTNAME=$SIGNING_IDENTITY
fi
if [ "$SIGNING_IDENTITY" == "" ]; then
  SIGNING_IDENTITY=$SIGNING_PROVIDER_SHORTNAME
fi

[ "$SIGNING_IDENTITY" == "" ] || git diff-index --quiet HEAD -- || fail "Signed release builds must not have unstaged changes!"

echo Updating dependencies
python3 $SOURCE_ROOT/setup-deps.py

echo Cleaning output directories
rm -rf $BUILD_FOLDER
rm -rf $INSTALLER_FOLDER
mkdir $BUILD_ROOT
mkdir $BUILD_FOLDER
mkdir $INSTALLER_FOLDER

# Enable LTO for official builds
export CFLAGS=-flto=thin
export CXXFLAGS=-flto=thin
export LDFLAGS=-flto=thin

echo Configuring the project
pushd $BUILD_FOLDER
qmake $SOURCE_ROOT/moonlight-qt.pro QMAKE_APPLE_DEVICE_ARCHS="x86_64 arm64" || fail "Qmake failed!"
popd

echo Compiling Moonlight in $BUILD_CONFIG configuration
pushd $BUILD_FOLDER
make -j$(sysctl -n hw.logicalcpu) $(echo "$BUILD_CONFIG" | tr '[:upper:]' '[:lower:]') || fail "Make failed!"
popd

echo Saving dSYM file
pushd $BUILD_FOLDER
dsymutil "$APP_BUNDLE/Contents/MacOS/Moonlight" -o Moonlight-$VERSION.dsym || fail "dSYM creation failed!"
cp -R Moonlight-$VERSION.dsym $INSTALLER_FOLDER || fail "dSYM copy failed!"
popd

echo Hiding libmimer
if [ -d "$QTDIR" ]; then
  mv "$QTDIR/macos/plugins/sqldrivers/libqsqlmimer.dylib" "$QTDIR/macos/plugins/sqldrivers/libqsqlmimer.dylib.hide"
fi

if [ "$SIGNING_IDENTITY" != "" ]; then
  echo Setting up entitlements
  if [ "$MOONLIGHT_PROVISION_PROFILE" == "" ]; then
    fail "Please set MOONLIGHT_PROVISION_PROFILE to the path to your .provisionprofile"
  fi
  cp $SOURCE_ROOT/app/deploy/macos/spatial-audio.entitlements "$APP_BUNDLE/Contents/Resources/spatial-audio.entitlements"
  cp $MOONLIGHT_PROVISION_PROFILE "$APP_BUNDLE/Contents/embedded.provisionprofile"
fi

echo Creating app bundle
EXTRA_ARGS=
if [ "$BUILD_CONFIG" == "Debug" ]; then EXTRA_ARGS="$EXTRA_ARGS -use-debug-libs"; fi
echo Extra deployment arguments: $EXTRA_ARGS
if [ "$SIGNING_IDENTITY" != "" ]; then
  macdeployqt "$APP_BUNDLE" $EXTRA_ARGS -qmldir=$SOURCE_ROOT/app/gui -codesign="$SIGNING_IDENTITY" -hardened-runtime -timestamp || fail "macdeployqt failed!"
else
  macdeployqt "$APP_BUNDLE" $EXTRA_ARGS -qmldir=$SOURCE_ROOT/app/gui -no-codesign || fail "macdeployqt failed!"
fi

mv "$QTDIR/macos/plugins/sqldrivers/libqsqlmimer.dylib.hide" "$QTDIR/macos/plugins/sqldrivers/libqsqlmimer.dylib"

# TODO: lots more Qt crap that can be removed
echo Removing unused Qt cruft
for plugin in geometryloaders multimedia sqldrivers; do
  if [ -d "$APP_BUNDLE/Contents/PlugIns/$plugin" ]; then
    rm -rf "$APP_BUNDLE/Contents/PlugIns/$plugin"
    echo "Removed Qt plugin: $plugin"
  fi
done

for framework in Qt3D* QtMultimedia QtShaderTools QtSql QtVirtualKeyboard QtVirtualKeyboardQml QtVirtualKeyboardSettings; do
  if [ -d "$APP_BUNDLE/Contents/Frameworks/$framework.framework" ]; then
    rm -rf "$APP_BUNDLE/Contents/Frameworks/$framework.framework"
    echo "Removed Qt framework: $framework"
  fi
done
rm -rf "$APP_BUNDLE/Contents/Frameworks/Qt3D*"

echo Removing dSYM files from app bundle
find "$APP_BUNDLE/" -name '*.dSYM' | xargs rm -rf

if [ "$SIGNING_IDENTITY" != "" ]; then
  echo Signing app bundle
  codesign --force --deep --verify --verbose --options runtime --timestamp \
    --entitlements "$APP_BUNDLE/Contents/Resources/spatial-audio.entitlements" \
    --sign "$SIGNING_IDENTITY" \
    "$APP_BUNDLE" || fail "Signing failed!"
  echo "App signature:"
  codesign -d --entitlements - -vvv "$APP_BUNDLE"
fi

echo Creating DMG
if [ "$SIGNING_IDENTITY" != "" ]; then
  create-dmg "$APP_BUNDLE" $INSTALLER_FOLDER --identity="$SIGNING_IDENTITY" --no-version-in-filename || fail "create-dmg failed!"
else
  create-dmg "$APP_BUNDLE" $INSTALLER_FOLDER --no-code-sign --no-version-in-filename
  case $? in
    0) ;;
    2) ;;
    *) fail "create-dmg failed!";;
  esac
fi

# Space in DMG filename is expected here
if [ "$NOTARY_KEYCHAIN_PROFILE" != "" ]; then
  echo Uploading to App Notary service
  xcrun notarytool submit --keychain-profile "$NOTARY_KEYCHAIN_PROFILE" --wait "$INSTALLER_FOLDER/Moonlight Metal.dmg" || fail "Notary submission failed"

  echo Stapling notary ticket to DMG
  xcrun stapler staple -v "$INSTALLER_FOLDER/Moonlight Metal.dmg" || fail "Notary ticket stapling failed!"
fi

mv "$INSTALLER_FOLDER/Moonlight Metal.dmg" $INSTALLER_FOLDER/Moonlight-$VERSION.dmg
echo Build successful
