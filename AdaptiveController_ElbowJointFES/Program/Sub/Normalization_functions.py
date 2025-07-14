def normalize(value,min_value,max_value):
    # Normalize the value
    return (value - min_value) / (max_value - min_value)


def scale_output(y_norm, min_val, max_val):
    # Scale normalized value back
    return y_norm * (max_val - min_val) + min_val

def normalize_list(input_list, min_value, max_value):
    # Initialize a list for normalization
    normalized_list = []

    # Normalize every item
    for item in input_list:
        normalized_value = normalize(item, min_value, max_value)
        normalized_list.append(normalized_value)

    return normalized_list