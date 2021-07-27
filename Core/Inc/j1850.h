// define J1850 VPW timing requirements in accordance with SAE J1850 standard
// all pulse width times in us
// transmitting pulse width
#define TX_SHORT	64	// Short pulse nominal time
#define TX_LONG		128	// Long pulse nominal time
#define TX_SOF		200	// Start Of Frame nominal time
#define TX_EOD		200	// End Of Data nominal time
#define TX_EOF		280	// End Of Frame nominal time
#define TX_BRK		300	// Break nominal time
#define TX_IFS		300	// Inter Frame Separation nominal time

// see SAE J1850 chapter 6.6.2.5 for preferred use of In Frame Respond/Normalization pulse
#define TX_IFR_SHORT_CRC	64// short In Frame Respond, IFR contain CRC
#define TX_IFR_LONG_NOCRC 128	// long In Frame Respond, IFR contain no CRC

// receiving pulse width
#define RX_SHORT_MIN	34	// minimum short pulse time
#define RX_SHORT_MAX	96	// maximum short pulse time
#define RX_LONG_MIN	96	// minimum long pulse time
#define RX_LONG_MAX	163	// maximum long pulse time
#define RX_SOF_MIN	123	// l'he posat a 153 (abans 163) minimum start of frame time
#define RX_SOF_MAX	279	// l'he posat a 249 (abans 239) maximum start of frame time
#define RX_EOD_MIN	163	// minimum end of data time
#define RX_EOD_MAX	239	// maximum end of data time
#define RX_EOF_MIN	239	// minimum end of frame time, ends at minimum IFS
#define RX_BRK_MIN	239	// minimum break time
#define RX_IFS_MIN	280	// minimum inter frame separation time, ends at next SOF

// see chapter 6.6.2.5 for preferred use of In Frame Respond/Normalization pulse
#define RX_IFR_SHORT_MIN	34// minimum short in frame respond pulse time
#define RX_IFR_SHORT_MAX	96// maximum short in frame respond pulse time
#define RX_IFR_LONG_MIN		96// minimum long in frame respond pulse time
#define RX_IFR_LONG_MAX		163// maximum long in frame respond pulse time




void j1850_break();