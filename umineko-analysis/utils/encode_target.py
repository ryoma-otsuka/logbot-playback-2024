def encode_target(
        df, target_column, target_class_string, secondary_class_string, 
        prev_targets, target_dict, binary=True):
    '''
    Encode target class int list and dictionary?
    
    Args
    ------
    target_column: str
        "label"
    target_class_string: str
        "foraging", "flying", "stationary", ...
    secondary_class_string: string
        a different class from the target class
    prev_targets: list

    target_dict: dict
    
    binary: bool

    '''

    df_mod = df.copy()
    targets = list(df_mod[target_column].unique())
    print('targets: ', targets)
    targets.sort()

    if prev_targets == None:
        prev_targets = []
    if target_dict == None:
        target_dict = {}

    if binary:
        prev_targets = [target_class_string, "others"]
        target_dict[target_class_string] = 0

        for t in targets:
            if t != target_class_string:
                target_dict[t] = 1

    else:
        if target_class_string not in prev_targets:
            prev_targets.append(target_class_string)
            target_dict[target_class_string] = 0

        if secondary_class_string is not None and secondary_class_string not in prev_targets:
            prev_targets.append(secondary_class_string)
            target_dict[secondary_class_string] = 1

        for t in targets:
            if t not in prev_targets:
                target_dict[t] = len(prev_targets)
                prev_targets.append(t)

    df_mod[target_column] = df_mod[target_column].replace(target_dict)
    return (df_mod, prev_targets, target_dict)
    