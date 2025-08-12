SECTIONS
{
	.param : {
        PROVIDE(_param_start = .);
        KEEP(*(.param))
        KEEP(*(.param.*))
        PROVIDE(_param_stop = .);
    }
    .log : {
        PROVIDE(_log_start = .);
        KEEP(*(.log))
        KEEP(*(.log.*))
        PROVIDE(_log_stop = .);
    }
}
INSERT AFTER .text;