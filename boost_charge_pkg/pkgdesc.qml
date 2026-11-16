import QtQuick 2.15

Item {
    property string pkgName: "Boost Charge"
    property string pkgDescriptionMd: "README.md"
    property string pkgLisp: "boost_charge.lisp"
    property string pkgQml: "ui.qml"
    property bool pkgQmlIsFullscreen: false
    property string pkgOutput: "boost_charge.vescpkg"

    // Motor controllers only. The firmware must have the boost-* extensions
    // (motor/boost_charge.c), which cannot be checked from here.
    function isCompatible (fwRxParams) {
        return fwRxParams.hwTypeStr().toLowerCase() == "vesc"
    }
}
