var group__group__usbfxstack__fx__utils__structs =
[
    [ "cy_stc_debug_config_t", "group__group__usbfxstack__fx__utils__structs.html#structcy__stc__debug__config__t", [
      [ "pBuffer", "group__group__usbfxstack__fx__utils__structs.html#afdd118b88960e29ebf4cc40341d4e713", null ],
      [ "traceLvl", "group__group__usbfxstack__fx__utils__structs.html#a2753d125b04964b54d0dffd32fbd3091", null ],
      [ "bufSize", "group__group__usbfxstack__fx__utils__structs.html#aff601bbda91c001a747d0226a49fa859", null ],
      [ "dbgIntfce", "group__group__usbfxstack__fx__utils__structs.html#afb980d5462e075ea188bc234fd2e3181", null ],
      [ "printNow", "group__group__usbfxstack__fx__utils__structs.html#afcf7f70aec37f878ef7c8a2227bfbb11", null ]
    ] ],
    [ "cy_stc_debug_recv_context_t", "group__group__usbfxstack__fx__utils__structs.html#structcy__stc__debug__recv__context__t", [
      [ "pReadBuffer", "group__group__usbfxstack__fx__utils__structs.html#aed4beee5ca2f1e8a093f2b49de9c233b", null ],
      [ "readDoneCb", "group__group__usbfxstack__fx__utils__structs.html#a408e1635ed5a9a9ef661f0ca9a738861", null ],
      [ "pUserCtxt", "group__group__usbfxstack__fx__utils__structs.html#a9988323318cabe95460889e2063f2a11", null ],
      [ "requestSize", "group__group__usbfxstack__fx__utils__structs.html#a1f1cfa300f05a9ae8983a7f11ec0d76c", null ],
      [ "readCount", "group__group__usbfxstack__fx__utils__structs.html#ac1b4f54834613b77f778a6a02d6001d0", null ]
    ] ],
    [ "cy_stc_debug_context_t", "group__group__usbfxstack__fx__utils__structs.html#structcy__stc__debug__context__t", [
      [ "pMsg", "group__group__usbfxstack__fx__utils__structs.html#a405348f6e12ed525df687b66066a32f7", null ],
      [ "bufSize", "group__group__usbfxstack__fx__utils__structs.html#ade884ddcf8a1c4b006d8805e57fdc2f2", null ],
      [ "rdPtr", "group__group__usbfxstack__fx__utils__structs.html#a3a376a1ce9a0110c03505b901352f370", null ],
      [ "wrPtr", "group__group__usbfxstack__fx__utils__structs.html#a1fdeb81e54be2ee426389c6e06038f44", null ],
      [ "intfc", "group__group__usbfxstack__fx__utils__structs.html#acd30e6974359fd83c45a8b2f4a9984ea", null ],
      [ "dbgLevel", "group__group__usbfxstack__fx__utils__structs.html#ae1f7ca7f29831e2d73e5c0a3640dee69", null ],
      [ "printNow", "group__group__usbfxstack__fx__utils__structs.html#a6621427b0e274d76237b02f7dd7f414c", null ],
      [ "maxDmaSize", "group__group__usbfxstack__fx__utils__structs.html#a07d41fc766b39aec6f7ecab4db4524b8", null ],
      [ "inProgress", "group__group__usbfxstack__fx__utils__structs.html#a16bc385bb7abe1c80b01d2a1850ce9fb", null ],
      [ "pDbgScb", "group__group__usbfxstack__fx__utils__structs.html#ac369bbda9264f5f74f758a9a66a2653e", null ],
      [ "dbgDmaDscr", "group__group__usbfxstack__fx__utils__structs.html#ac1df230e48e867b246e619f061cb893d", null ],
      [ "dbgRcvInfo", "group__group__usbfxstack__fx__utils__structs.html#aa50d25a37633c8b64f4af7e2377cf361", null ]
    ] ],
    [ "cy_stc_usbfs_epinfo_t", "group__group__usbfxstack__fx__utils__structs.html#structcy__stc__usbfs__epinfo__t", [
      [ "is_out", "group__group__usbfxstack__fx__utils__structs.html#a966f9ea6b5a92fc496723d6c28664aeb", null ],
      [ "toggle", "group__group__usbfxstack__fx__utils__structs.html#a91836eada88671929159d5b2a8e63c9c", null ],
      [ "enabled", "group__group__usbfxstack__fx__utils__structs.html#a547a11bae5415830a4316b97477cac01", null ]
    ] ],
    [ "cy_stc_usb_setup_pkt_t", "group__group__usbfxstack__fx__utils__structs.html#structcy__stc__usb__setup__pkt__t", [
      [ "attrib", "group__group__usbfxstack__fx__utils__structs.html#a3b42cd6f771a0d30b74e147539475145", null ],
      [ "cmd", "group__group__usbfxstack__fx__utils__structs.html#a23613b19917be41de399282773aa8ab4", null ],
      [ "value", "group__group__usbfxstack__fx__utils__structs.html#a9fe14651158f822b16377bdce9f39fe8", null ],
      [ "index", "group__group__usbfxstack__fx__utils__structs.html#a93a41def1627b3b03cbb4b3354518924", null ],
      [ "length", "group__group__usbfxstack__fx__utils__structs.html#a41676accf2922495e1b68c517dfeba0c", null ]
    ] ],
    [ "cy_usbfs_devhandle_t", "group__group__usbfxstack__fx__utils__structs.html#structcy__usbfs__devhandle__t", [
      [ "dev_stat", "group__group__usbfxstack__fx__utils__structs.html#ac7da0798096cb515ea07c8426d74d97e", null ],
      [ "state", "group__group__usbfxstack__fx__utils__structs.html#acd99f3ce9a4ba452ad98c084132adb64", null ],
      [ "prev_state", "group__group__usbfxstack__fx__utils__structs.html#a9f608a5e59033d256746cb2ed6f6b944", null ],
      [ "setup_pkt", "group__group__usbfxstack__fx__utils__structs.html#a40403277b0817d33bf226446526f9781", null ],
      [ "ep0_state", "group__group__usbfxstack__fx__utils__structs.html#ababf8b6e814a2f393d4d846887ff9643", null ],
      [ "ep0_toggle", "group__group__usbfxstack__fx__utils__structs.html#a87fee350190e28b9e45cbafe7cf90093", null ],
      [ "ep0_buffer", "group__group__usbfxstack__fx__utils__structs.html#a437ff0e101e3186ed38e9fe687b6c386", null ],
      [ "ep0_length", "group__group__usbfxstack__fx__utils__structs.html#a1e0b17be19c4c20a4b5c59feb1c71d79", null ],
      [ "ep0_zlp_rqd", "group__group__usbfxstack__fx__utils__structs.html#a5d58971bb53037757d6fcc789146e484", null ],
      [ "ep0_last", "group__group__usbfxstack__fx__utils__structs.html#a9a84938410830e43d635015c3fff03cc", null ],
      [ "active_cfg", "group__group__usbfxstack__fx__utils__structs.html#aab53799365740163ef8d327b1e6cd8d7", null ],
      [ "ep_handle", "group__group__usbfxstack__fx__utils__structs.html#a27a22a11a53ccfcc1a9e804e59e8495d", null ],
      [ "cdcConfig", "group__group__usbfxstack__fx__utils__structs.html#a42485749e4c77b063f9aa234b8e4c541", null ],
      [ "cdcRecvEnabled", "group__group__usbfxstack__fx__utils__structs.html#ab8badab835b90a855216bb255ee65ab4", null ]
    ] ],
    [ "cy_stc_usbhs_dma_desc_t", "group__group__usbfxstack__fx__utils__structs.html#structcy__stc__usbhs__dma__desc__t", [
      [ "dscrArray", "group__group__usbfxstack__fx__utils__structs.html#ab1308e72cdfc095b1a51030d639a4f56", null ]
    ] ],
    [ "cy_stc_app_endp_dma_set_t", "group__group__usbfxstack__fx__utils__structs.html#structcy__stc__app__endp__dma__set__t", [
      [ "pDwStruct", "group__group__usbfxstack__fx__utils__structs.html#aec6909da925d63e7621189e0804ac8bb", null ],
      [ "dmaXferDscr", "group__group__usbfxstack__fx__utils__structs.html#a9ed67ffd20726e3a77bf50f3270cecf4", null ],
      [ "hbDmaChannel", "group__group__usbfxstack__fx__utils__structs.html#a4ae4e5fb4d87f9815365f04ce1cac6d2", null ],
      [ "maxPktSize", "group__group__usbfxstack__fx__utils__structs.html#af301de331d45ca2b3f6e6b557bb0481f", null ],
      [ "channel", "group__group__usbfxstack__fx__utils__structs.html#a0f2ddab1e3b16994da7cb54abe768faf", null ],
      [ "epNumber", "group__group__usbfxstack__fx__utils__structs.html#af7bfd56b08f78fab710ee2c3a049350f", null ],
      [ "epDir", "group__group__usbfxstack__fx__utils__structs.html#a9074a9e5ba96f95ebc1ffbe0518108aa", null ],
      [ "valid", "group__group__usbfxstack__fx__utils__structs.html#a05d4611bb239e6f7312198e37fc4e860", null ],
      [ "firstRqtDone", "group__group__usbfxstack__fx__utils__structs.html#adcbbfcdd3e1bef7112d0e7a160643626", null ],
      [ "endpType", "group__group__usbfxstack__fx__utils__structs.html#a01e07d984f7acc0e328fe7126565e663", null ]
    ] ]
];