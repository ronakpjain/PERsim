SUMMARY = "PERsim SocketCAN to localhost-host TCP gateway"
DESCRIPTION = "Classic CAN guest bridge for the optional virtualized CM5 TV path"
LICENSE = "CLOSED"

FILESEXTRAPATHS:prepend := "${THISDIR}/../../../:"
SRC_URI = "file://vcan_bridge.c"
S = "${WORKDIR}"

do_compile() {
    ${CC} ${CFLAGS} -std=c11 -Wall -Wextra ${WORKDIR}/vcan_bridge.c ${LDFLAGS} -o persim-vcan
}

do_install() {
    install -d ${D}${bindir}
    install -m 0755 persim-vcan ${D}${bindir}/persim-vcan
}
