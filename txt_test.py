def write_txt():
    name = 'taa'
    name += '.txt'
    with open(name, 'a') as f:
        i = 0
        while i<5:
            f.write('asdf\n')
            i += 1
write_txt()