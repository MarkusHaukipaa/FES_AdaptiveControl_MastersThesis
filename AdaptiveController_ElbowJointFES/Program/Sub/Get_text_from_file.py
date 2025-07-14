def get_text(file_path, target_line_num, start_pos, end_pos):
    """Opens a file and outputs the wanted line
        If not found, returns nothing
    """

    with open(file_path, "r", encoding="utf-8") as file:
        lines = file.readlines()
        # Check if the specified line number is within range
        if 0 <= target_line_num - 1 < len(lines):
            # Slice a substring from a given line
            target_line = lines[target_line_num - 1]
            extracted_text = target_line[start_pos:end_pos]
        else:
            print(f"Line {target_line_num} Out of range of file")
            exit()
    return extracted_text