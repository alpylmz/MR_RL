import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

GAUSSIAN_DATA_COUNT = 5
SAMPLE_COUNT = 100

def arrange_data(data_by_image):
    # we need to go backwards
    # we need to take a look at TrackObjects_ParentObjectNumber_15, to see what the parent is
    # and ObjectNumber to see what the object's number is
    datas = []
    arr_index = 0
    
    # take the data_by_image so that every row that has image number 0 is in the dataframe, others are not
    first_image = data_by_image[data_by_image['ImageNumber'] == 25]
    for i, object in first_image.iterrows():
        #print(i, object)
        temp_dict = {}
        temp_dict['ImageNumber'] = 25
        temp_dict['ObjectNumber'] = object['ObjectNumber']
        temp_dict["center_x"] = object['AreaShape_Center_X']
        temp_dict["center_y"] = object['AreaShape_Center_Y']
        
        datas.append([])
        datas[arr_index].append(temp_dict)
        
        object_number = object['TrackObjects_ParentObjectNumber_15']
    
        # traversing everything image by image would be better, but it would be a lot more complicated to write
        # just do it this way for now
        
        j = 24
        #print("\n\n\n\n\n")
        while j > 0:
            #print("current image number: ", j)
            #print("current object number: ", object_number)
            temp_dict = {}
            temp_object = data_by_image.loc[(data_by_image['ImageNumber'] == j) & (data_by_image['ObjectNumber'] == object_number)]
            # if the data is the empty dataframe, then we have reached the end of the chain
            if temp_object.empty:
                break
            
            temp_dict['ImageNumber'] = j
            temp_dict['ObjectNumber'] = object_number
            temp_dict["center_x"] = temp_object['AreaShape_Center_X'].iloc[0]
            temp_dict["center_y"] = temp_object['AreaShape_Center_Y'].iloc[0]
            
            datas[arr_index].append(temp_dict)
            #print(temp_dict)
            
            object_number = temp_object['TrackObjects_ParentObjectNumber_15'].iloc[0]
            j -= 1
    
        arr_index += 1
    
    return datas

def plot_analyze(datas):
    returned = []
    for data in datas:
        for i in range(len(data) - 2):
            # calculate the speed of the object
            speed_1 = [
                data[i+1]["center_x"] - data[i]["center_x"],
                data[i+1]["center_y"] - data[i]["center_y"]
            ]
            speed_2 = [
                data[i+2]["center_x"] - data[i+1]["center_x"],
                data[i+2]["center_y"] - data[i+1]["center_y"]
            ]
            
            returned.append([np.linalg.norm(speed_1), np.linalg.norm(speed_2)])
    
    for range_previous in np.arange(0.0, 5.0, 0.1):
        range_next = range_previous + 0.1
        add_to_plot = []
        for x in returned:
            previous = x[0]
            next = x[1]
            
            if previous > range_previous and previous < range_next:
                add_to_plot.append(next)
                
        plt.clf()
            
        # filtered out the ones that are not in wanted range
        # plot the others as a histogram
        plt.hist(add_to_plot, bins=100)
        
        plt.xlim(0, 5)
        
        plt.xlabel("Speeds for the next second")
        plt.ylabel("Count")
        
        plt.title(f"Speeds for the next time instance if the previous speed is between {range_previous} and {range_next}")
        
        plt.savefig(f"speeds_for_next_time_instance_{range_previous}_{range_next}.png")  

def score_gaussian(x, mu, std):
    # sample from the gaussian distribution, check the difference between the sample and the actual value
    # return the average of the differences
    
    avg_diff = 0
    for _ in range(SAMPLE_COUNT):
        sample = np.random.normal(mu, std)
        diff = abs(sample - x)

        avg_diff += diff
        
    avg_diff /= SAMPLE_COUNT
    return avg_diff

def gaussian_prediction(data):
    # sort it by image number key
    data.sort(key=lambda x: x['ImageNumber'])

    # get the first GAUSSIAN_DATA_COUNT number of data points, do a gaussian fit, then 
    # get the next GAUSSIAN_DATA_COUNT number of data points, do a gaussian fit, and so on
    
    scores = []
    for i in range(0, len(data) - GAUSSIAN_DATA_COUNT - 1):
        
        curr_data = data[i:i+GAUSSIAN_DATA_COUNT]
        
        xs = [x['center_x'] for x in curr_data]
        ys = [x['center_y'] for x in curr_data]
        #print("xs: ", xs)
        #print("ys: ", ys)
        
        velocity_x = np.diff(xs)
        velocity_y = np.diff(ys)
        speeds = np.sqrt(np.square(velocity_x) + np.square(velocity_y))
        
        mu, std = norm.fit(speeds)
        
        next_velocity = [
            data[i+GAUSSIAN_DATA_COUNT]["center_x"] - data[i+GAUSSIAN_DATA_COUNT-1]["center_x"],
            data[i+GAUSSIAN_DATA_COUNT]["center_y"] - data[i+GAUSSIAN_DATA_COUNT-1]["center_y"]
        ]
        next_speed = np.linalg.norm(next_velocity)
        
        score = score_gaussian(next_speed, mu, std)
        scores.append(score)
        
    # return the average of the scores
    sum_score = 0
    for score in scores:
        sum_score += score
    if len(scores) == 0:
        return None
    return sum_score / len(scores)


if __name__ == "__main__":
    df = pd.read_csv('../../IdentifyPrimaryObjects.csv')
    
    datas = arrange_data(df)
    
    plot_analyze(datas)
    exit(1)
    
    avg_score = 0
    for data in datas:
        score = gaussian_prediction(data)
        if score is not None:
            avg_score += score
            
    avg_score /= len(datas)
    print("avg_score: ", avg_score)
    
    
    
    