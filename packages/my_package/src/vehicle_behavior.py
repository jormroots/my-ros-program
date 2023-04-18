class behavior():
    def gone_offroad(bits_block):
        return bits_block == '00000000'
    def going_forward(bits_block):
        return bits_block == '00011000'
    def delayed_turn(bits_block):
        return bits_block == '11011000'
    def long_right_turn(bits_block):
        return bits_block == '11111000'
    def long_left_turn(bits_block):
        return bits_block == '00011111'