import os
import pandas as pd
import datetime

def saveDataToCSV(data, bug_flag, folder_path, SES_flag, recruitment = "",
                  controller_name = ""):

    df = pd.DataFrame(data)

    # Form a path to current folder to a Result_Folder

    # Make it if not made yet
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # File Naming
    if not bug_flag:
        result_file_name = (f'{recruitment}_{controller_name}_'
                            f'{datetime.datetime.now().strftime("%d%m_%H%M")}.csv')
    else:
        result_file_name = (f'BUG{recruitment}_{controller_name}_'
                            f'{datetime.datetime.now().strftime("%d%m_%H%M")}.csv')

    csv_filename = os.path.join(folder_path, result_file_name)

    # SAVING
    df.to_csv(csv_filename, index=False)
    print("Saving complete to File:")
    print(csv_filename)