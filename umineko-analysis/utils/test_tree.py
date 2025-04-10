def tree(acc_mag_mean, acc_mag_1c, acc_mag_mc, acc_mag_var, acc_mag_energy, acc_mag_kurtosis, acc_mag_crest, acc_mag_RMS):
    if acc_mag_crest <= 0.6229148173371786:
        if acc_mag_crest <= 0.45141414907431915:
            if acc_mag_crest <= 0.21079075585583174:
                if acc_mag_crest <= 0.18464216232850303:
                    if acc_mag_crest <= 0.05576565269196411:
                        return 1 # stationary
                    else:  # if acc_mag_crest > 0.05576565269196411
                        return 1 # stationary
                else:  # if acc_mag_crest > 0.18464216232850303
                    if acc_mag_kurtosis <= 11.172124346312481:
                        return 1 # stationary
                    else:  # if acc_mag_kurtosis > 11.172124346312481
                        return 1 # stationary
            else:  # if acc_mag_crest > 0.21079075585583174
                if acc_mag_mc <= 8.279699635356526:
                    if acc_mag_mc <= 7.447097010187385:
                        return 1 # stationary
                    else:  # if acc_mag_mc > 7.447097010187385
                        return 1 # stationary
                else:  # if acc_mag_mc > 8.279699635356526
                    if acc_mag_kurtosis <= 6.33432568943421:
                        return 1 # stationary
                    else:  # if acc_mag_kurtosis > 6.33432568943421
                        return 1 # stationary
        else:  # if acc_mag_crest > 0.45141414907431915
            if acc_mag_RMS <= 1.2474981394832996:
                if acc_mag_RMS <= 1.1239222618799805:
                    if acc_mag_mc <= 7.362419893202567:
                        return 1 # stationary
                    else:  # if acc_mag_mc > 7.362419893202567
                        return 1 # stationary
                else:  # if acc_mag_RMS > 1.1239222618799805
                    if acc_mag_crest <= 0.514728035687301:
                        return 1 # stationary
                    else:  # if acc_mag_crest > 0.514728035687301
                        return 0 # flying
            else:  # if acc_mag_RMS > 1.2474981394832996
                if acc_mag_mc <= 6.3847267960127105:
                    if acc_mag_crest <= 0.5775411015469464:
                        return 0 # flying
                    else:  # if acc_mag_crest > 0.5775411015469464
                        return 0 # flying
                else:  # if acc_mag_mc > 6.3847267960127105
                    if acc_mag_crest <= 0.5679474306994543:
                        return 1 # stationary
                    else:  # if acc_mag_crest > 0.5679474306994543
                        return 0 # flying
    else:  # if acc_mag_crest > 0.6229148173371786
        if acc_mag_kurtosis <= 12.973689102558513:
            if acc_mag_mc <= 10.121539143902035:
                if acc_mag_kurtosis <= 5.566636476535482:
                    if acc_mag_mc <= 7.454518176826889:
                        return 0 # flying
                    else:  # if acc_mag_mc > 7.454518176826889
                        return 1 # stationary
                else:  # if acc_mag_kurtosis > 5.566636476535482
                    if acc_mag_var <= 1.165256407225706:
                        return 1 # stationary
                    else:  # if acc_mag_var > 1.165256407225706
                        return 1 # stationary
            else:  # if acc_mag_mc > 10.121539143902035
                if acc_mag_var <= 4.924182967835713:
                    if acc_mag_kurtosis <= 8.23279497926137:
                        return 1 # stationary
                    else:  # if acc_mag_kurtosis > 8.23279497926137
                        return 1 # stationary
                else:  # if acc_mag_var > 4.924182967835713
                    if acc_mag_mean <= 4.875479680854032:
                        return 1 # stationary
                    else:  # if acc_mag_mean > 4.875479680854032
                        return 1 # stationary
        else:  # if acc_mag_kurtosis > 12.973689102558513
            if acc_mag_crest <= 0.9908165399084127:
                if acc_mag_var <= 0.03381158816521909:
                    if acc_mag_energy <= 0.6924729156893883:
                        return 1 # stationary
                    else:  # if acc_mag_energy > 0.6924729156893883
                        return 1 # stationary
                else:  # if acc_mag_var > 0.03381158816521909
                    if acc_mag_var <= 0.09998910404037126:
                        return 1 # stationary
                    else:  # if acc_mag_var > 0.09998910404037126
                        return 1 # stationary
            else:  # if acc_mag_crest > 0.9908165399084127
                if acc_mag_crest <= 1.2892180812842213:
                    if acc_mag_crest <= 1.0388268709224056:
                        return 1 # stationary
                    else:  # if acc_mag_crest > 1.0388268709224056
                        return 1 # stationary
                else:  # if acc_mag_crest > 1.2892180812842213
                    if acc_mag_crest <= 1.5700820002376188:
                        return 1 # stationary
                    else:  # if acc_mag_crest > 1.5700820002376188
                        return 1 # stationary
