import numpy as np
import pandas as pd
from copy import deepcopy
from scipy.stats import norm
import matplotlib.pyplot as plt

GAUSSIAN_DATA_COUNT = 5
SAMPLE_COUNT = 100

pd.options.display.max_columns = 100

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
        
def gaussian_fit(thetas):
    scores = []
    for sublist in thetas:
        #print("sublist: ", sublist)
        # tale the previous GAUSSIAN_DATA_COUNT number of data points, do a gaussian fit
        for i in range(0, len(sublist) - GAUSSIAN_DATA_COUNT - 1):
            curr_data = sublist[i:i+GAUSSIAN_DATA_COUNT]
            #print("curr_data: ", curr_data)
            mu, std = norm.fit(curr_data)
            #print("mu: ", mu, "std: ", std)
            
            next_theta = sublist[i+GAUSSIAN_DATA_COUNT]
            
            score = score_gaussian(next_theta, mu, std)
            
            scores.append(score)
            
    avg_score = np.mean(scores)
    return avg_score
    
def predict_next_theta(thetas):
    mu, std = norm.fit(thetas)
    
    next_theta = np.random.normal(mu, std)
    return next_theta

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
        #print("velocity x: ", velocity_x)
        #print("velocity y: ", velocity_y)
        
        velocity_theta = np.arctan2(velocity_y, velocity_x)
        # limit the range of the theta to be between -pi and pi
        for j in range(len(velocity_theta)):
            if velocity_theta[j] > np.pi:
                velocity_theta[j] -= 2*np.pi
            elif velocity_theta[j] < -np.pi:
                velocity_theta[j] += 2*np.pi
        #print("velocity theta: ", velocity_theta)
        velocity_theta_change = [velocity_theta[i] - velocity_theta[i-1] for i in range(1, len(velocity_theta))]
        #print("velocity theta change: ", velocity_theta_change)
        next_velocity_theta = np.arctan2(
            data[i+GAUSSIAN_DATA_COUNT]['center_y'] - data[i+GAUSSIAN_DATA_COUNT-1]['center_y'],
            data[i+GAUSSIAN_DATA_COUNT]['center_x'] - data[i+GAUSSIAN_DATA_COUNT-1]['center_x']
        )
        
        mu, std = norm.fit(velocity_theta_change)
        print("mu: ", mu, "std: ", std)
        
        score = score_gaussian(next_velocity_theta, mu, std)
        scores.append(score)
        
    # return the average of the scores
    return np.mean(scores)
    
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

def analyze_data(thetas):
    
    previous_deviation = 0.14
    
    for previous_origin in [x for x in np.arange(-3.00, 3.00, previous_deviation)]:
        all_data = []
        for sublist in thetas:
            i = 0
            all_data.append([])
            while i < (len(sublist) - 2):
                # find the theta diff
                theta_diff = sublist[i+1] - sublist[i]
                if theta_diff > np.pi:
                    theta_diff -= 2*np.pi
                elif theta_diff < -np.pi:
                    theta_diff += 2*np.pi
                    
                if theta_diff > (previous_origin - previous_deviation) and theta_diff < (previous_origin + previous_deviation):
                    #print("theta diff: ", theta_diff)
                    theta_diff2 = sublist[i+2] - sublist[i+1]
                    if theta_diff2 > np.pi:
                        theta_diff2 -= 2*np.pi
                    elif theta_diff2 < -np.pi:
                        theta_diff2 += 2*np.pi
                    all_data[-1].append(theta_diff2)
                
                i += 1
    
        # flatten the list
        all_data = [item for sublist in all_data for item in sublist]
        # plot a bar graph of the data
        plt.hist(all_data)
        plt.title(f"Theta occurences with previous theta between {previous_origin - previous_deviation} and {previous_origin + previous_deviation}")
        plt.ylabel('Number of occurrence')
        plt.xlabel("Theta difference (radians)")
        plt.savefig(f"theta_occurences_{previous_origin}.png")
        plt.clf()

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
            plt.plot(x, y, 'bo-')
            
            # now, get the theta data corresponding to the position data
            temp_theta = [theta[i][j+k] for k in range(GAUSSIAN_DATA_COUNT)]
            print("theta: ", temp_theta)
            # now get the difference between consecutive theta
            temp_theta = [temp_theta[k+1] - temp_theta[k] for k in range(len(temp_theta)-1)]
            print("theta diff: ", temp_theta)
            
            mu, std = norm.fit(temp_theta)
            
            next_theta_diff = np.random.normal(mu, std)
            next_theta = theta[i][j+GAUSSIAN_DATA_COUNT-1] + next_theta_diff
            
            # draw a line from the last position, to a direction of next_theta, with a length of 1
            next_position = (x[-1] + np.cos(next_theta), y[-1] + np.sin(next_theta))
            next_position_mean = (x[-1] + np.cos(mu), y[-1] + np.sin(mu))
            
            print("x: ", (x[-1], y[-1]))
            print("next position: ", next_position)

            plt.title(f"Std: {std}")
            plt.plot([x[-1], next_position_mean[0]], [y[-1], next_position_mean[1]], 'ro-')
            #plt.plot([x[-1], next_position[0]], [y[-1], next_position[1]], 'ro-')
            
            # draw the last x and y
            plt.plot(last_x, last_y, 'go')
            
            j += 1
            
            plt.savefig(f"prediction_std: {std}.png")
        
        i += 1
    
    

if __name__ == '__main__':
    df = pd.read_csv('../../IdentifyPrimaryObjects.csv')
    #data_by_image = df.groupby('ImageNumber').agg(pd.DataFrame)
    # set the index to be the image number
    # groupby the dataframe by using imagenumber, and make the result a list of dataframes, and then reset the index
    datas = arrange_data(df)
    
    theta, position = extract_theta_and_positions(datas)
    
    predict_and_plot(position, theta)

    
    
    