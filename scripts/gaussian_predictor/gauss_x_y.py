import numpy as np
import pandas as pd
from scipy.stats import norm
import matplotlib.pyplot as plt

GAUSSIAN_DATA_COUNT = 5
SAMPLE_COUNT = 100
CONFIDENCE_PERCENTAGE = 0.70

pd.options.display.max_columns = 100

def score_gaussian(actual_x, actual_y, mu_x, std_x, mu_y, std_y, last_x, last_y):
    # sample from the gaussian distribution, check the difference between the sample and the actual value
    # return the average of the differences
    
    avg_diff = 0
    for _ in range(SAMPLE_COUNT):
        sample_x = np.random.normal(mu_x, std_x)
        sample_y = np.random.normal(mu_y, std_y)
        sampled_position = np.array([
            sample_x + last_x,
            sample_y + last_y
        ])
        
        dist = np.linalg.norm(sampled_position - np.array([actual_x, actual_y]))
        
        avg_diff += dist
        
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
        last_x = xs[-1]
        last_y = ys[-1]
        
        xs = xs[:-1]
        ys = ys[:-1]
        
        velocity_x = np.diff(xs)
        velocity_y = np.diff(ys)
        
        velocity_x_mu, velocity_x_std = norm.fit(velocity_x)
        velocity_y_mu, velocity_y_std = norm.fit(velocity_y)
        
        
        score = score_gaussian(last_x, last_y, velocity_x_mu, velocity_x_std, velocity_y_mu, velocity_y_std, last_x, last_y)
        scores.append(score)
        
    # return the average of the scores
    sum_score = 0
    for score in scores:
        sum_score += score
    if len(scores) == 0:
        return None
    return sum_score / len(scores)
    
def extract_theta_and_positions(datas):
    theta_data = []
    position_data = []
    
    for data in datas:
        # sort it by image number key
        data.sort(key=lambda x: (x['ImageNumber'], x['ObjectNumber']))
        theta_data.append([])
        # to find the velocity, we need to find the difference between the center_x and center_y of the current image and the previous image
        i = 0
        #print("\n\n\n\n\n\noriginal data:")
        #print([(data[i]['center_x'], data[i]['center_y']) for i in range(len(data))])
        position_data.append([(data[i]['center_x'], data[i]['center_y']) for i in range(len(data))])
        while i < (len(data) - 1):
            
            velocity_x = data[i+1]['center_x'] - data[i]['center_x']
            velocity_y = data[i+1]['center_y'] - data[i]['center_y']
            
            # find theta
            theta = np.arctan2(velocity_y, velocity_x)
            # limit the theta to be between -pi and pi
            if theta > np.pi:
                theta -= 2*np.pi
            elif theta < -np.pi:
                theta += 2*np.pi
            
            theta_data[-1].append(theta)
            
            i += 1
        
        #print("position data: ", position_data[-1])
    
    return theta_data, position_data
    
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

def predict_and_plot(position, theta):
    
    i = 0
    j = 0
    while i < len(position):
        
        # position's length is n, theta's length is n-1
        # take the GAUSSIAN_DATA_COUNT number of thetas, and the GAUSSIAN_DATA_COUNT+1 number of positions
        # plot the positions first
        j = 0
        while j < len(position[i]) - GAUSSIAN_DATA_COUNT:
            plt.clf()
            x = [position[i][j+k][0] for k in range(GAUSSIAN_DATA_COUNT+1)]
            y = [position[i][j+k][1] for k in range(GAUSSIAN_DATA_COUNT+1)]
            # delete the last element of x and y
            last_x = x[-1]
            last_y = y[-1]
            
            x = x[:-1]
            y = y[:-1]
            print("positions: ", x, y)
            plt.plot(x, y, 'bo-')
            
            x_speeds = []
            y_speeds = []
            for k in range(len(x)-1):
                x_speed = x[k+1] - x[k]
                x_speeds.append(x_speed)
                y_speed = y[k+1] - y[k]
                y_speeds.append(y_speed)
                
            x_speed_mu, x_speed_std = norm.fit(x_speeds)
            y_speed_mu, y_speed_std = norm.fit(y_speeds)
            
            next_position_mean = [
                x[-1] + x_speed_mu,
                y[-1] + y_speed_mu
            ]
            
            plt.plot([x[-1], next_position_mean[0]], [y[-1], next_position_mean[1]], 'ro-')
            
            #plt.plot([x[-1], next_position_mean[0]], [y[-1], next_position_mean[1]], 'ro-')
            #plt.plot([x[-1], next_position[0]], [y[-1], next_position[1]], 'ro-')
            
            # draw the last x and y
            plt.plot(last_x, last_y, 'go')
            
            j += 1
            
            #plt.savefig(f"speed_prediction_{i}_{j}_.png")
            plt.show()
        i += 1
    
    
if __name__ == '__main__':
    df = pd.read_csv('../../IdentifyPrimaryObjects.csv')
    
    # conver the data into a list of dictionaries
    datas = arrange_data(df)
    #print("data: ", datas)
    """
    avg_score = 0
    for data in datas:
        score = gaussian_prediction(data)
        if score is None:
            continue
        avg_score += score
    avg_score /= len(datas)
    print("average score: ", avg_score)
    """    
     
    theta, position = extract_theta_and_positions(datas)
    
    predict_and_plot(position, theta)
    
    
    