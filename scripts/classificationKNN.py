#!/usr/bin/env python

import numpy as np
from scipy.signal import butter, lfilter, freqz, filtfilt
from scipy.stats import skew
from scipy.stats import kurtosis
import matplotlib.pyplot as plt
from scipy.fftpack  import fft, fftfreq
import pandas as pd

import seaborn as sns
from matplotlib.colors import ListedColormap
from sklearn import neighbors, datasets
from sklearn.metrics import accuracy_score, precision_score, confusion_matrix
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.model_selection import cross_val_score,StratifiedKFold
from sklearn.decomposition import PCA

n_neighbors = 10

def load_csv(path):
    df = pd.read_csv(path,header=None)
    data = df.to_numpy()
    return data


filetrainData = '/home/jrluser/my_workspace/controllers/lipm_walking_controller/scripts/TrainData.csv'
filetestData = '/home/jrluser/my_workspace/controllers/lipm_walking_controller/scripts/TestData.csv'
trainData = load_csv(filetrainData)
XtrainData = trainData[:,0:6]
ytrainData = trainData[:,6]
testData = load_csv(filetestData)
XtestData = testData[:,0:6]
ytestData = testData[:,6]
X = np.append(XtrainData, XtestData, axis=0)
y = np.append(ytrainData, ytestData, axis=0)
# print(XtrainData.shape)

# X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.15)

# Scale data
scaler = StandardScaler()
scaler.fit(XtrainData)
XtrainData = scaler.transform(XtrainData)
XtestData = scaler.transform(XtestData)
   
# we create an instance of Neighbours Classifier and fit the data.
clf = neighbors.KNeighborsClassifier(n_neighbors, weights="distance", metric="euclidean", algorithm="kd_tree")
clf.fit(XtrainData, ytrainData)

Z = clf.predict(XtestData)

# print(Z)
# print(ytestData)

# Display some metrics
#  Refer to https://scikit-learn.org/stable/modules/model_evaluation.html#classification-metrics

y_true = ytestData
y_pred = Z
print("Accuracy:", accuracy_score(y_true, y_pred))
cf_matrix = confusion_matrix(y_true, y_pred)
sns.heatmap(cf_matrix, annot=True, cmap='Blues')
plt.show()
# print("Precision score:", precision_score(y_true, y_pred, average=None))
print("Precision score Weithed:", precision_score(y_true, y_pred, average='weighted'))

# Perform cross-validation: K-Fold Cross-Validation
# Refer to https://scikit-learn.org/stable/modules/cross_validation.html

pipeline = Pipeline([('scaler', StandardScaler()), ('KNN', KNeighborsClassifier(n_neighbors, weights="distance", metric="euclidean", algorithm="kd_tree"))])
stratifiedkf=StratifiedKFold(n_splits=10)
score=cross_val_score(pipeline,X,y,cv=stratifiedkf, scoring="accuracy")
print("%0.2f accuracy with a standard deviation of %0.2f" % (score.mean(), score.std()))

# PCA analysis
pca = PCA(n_components=5)
scaler = StandardScaler()
scaler.fit(X)
scaled_X = scaler.transform(X)
pc = pca.fit_transform(scaled_X)
pcaDF = pd.DataFrame(data=pc)
# print(pca.explained_variance_)
# print(pca.explained_variance_ratio_)
height = pca.explained_variance_ratio_
bars = ('A', 'B', 'C', 'D', 'E')
y_pos = np.arange(len(bars))
plt.bar(y_pos, height)
plt.show()
print(height)

# Fit KNN to PCA components
pipeline_pca = KNeighborsClassifier(11, weights="distance", metric="euclidean", algorithm="kd_tree")
pipeline_pca.fit(pc[:,0:2], y)

# Create color maps
colors = ["orange", "cyan", "cornflowerblue", "green", "purple", "blue", "red", "magenta"]
markers = [">", "o", "s", "*", "x", "P", "+", "d"]
targets = list(set(y))

# Visualising the Training set results
X1, X2 = np.meshgrid(np.arange(start = pc[:, 0].min() - 0.02, stop = pc[:, 0].max() + 0.02, step = 0.01),
                     np.arange(start = pc[:, 1].min() - 0.02, stop = pc[:, 1].max() + 0.02, step = 0.01))

Z = pipeline_pca.predict(np.array([X1.ravel(), X2.ravel()]).T).reshape(X1.shape)
new_list = []
url_set = set()
for items in Z:
    for item in items:
        if item not in url_set:
            url_set.add(item)
            new_list.append(item)
        else:
            pass
# print(url_set)

fig, ax = plt.subplots()
# To be able to plot elt 12 replace it by id 8 because in contourf levels arguments must be a continuous array in an increasing order
Z[Z == 12.0] = 8.0

cs = plt.contourf(X1, X2, Z, alpha=0.15, colors=colors, levels=list(range(len(targets)+1)))
proxy = [plt.Rectangle((0,0),1,1,fc = ppc.get_facecolor()[0])  for ppc in cs.collections]
for target, color, marker in zip(targets,colors,markers):
    indicesToKeep = y == target
    plt.scatter(pcaDF.loc[indicesToKeep, 0], pcaDF.loc[indicesToKeep, 1], c=color, s=50, marker=marker, label=target)
    
plt.xlim(X1.min(), X1.max())
plt.ylim(X2.min(), X2.max())
plt.title('Logistic Regression (Training set)')
plt.xlabel('PC1')
plt.ylabel('PC2')
leg = plt.legend(loc=(1.03,0))
ax.add_artist(leg)
plt.legend(proxy, targets)
plt.show()