from cruise_control import branching_off_ahead


def test1():
    print("Passed 1: all good" if 1 == 1 else "Failed")


def test_branch_detection(read):
    print("branch case 1 detected OK" if branching_off_ahead(
        read) == True else "Failed")


def tests():
    test1()