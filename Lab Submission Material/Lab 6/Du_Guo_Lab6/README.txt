1.Please make sure the packages in requirements.txt has already been installed in your python environment.

2.Please put the models (cnn_all.pth, cnn.pth, cnn_23.pth), predict.py, crop_img.py ALONGSIDE with the test dataset folder, and make sure there is a labels.txt (the target labels of all the images) in test dataset folder as well. The document tree should be like:

--predict.py
--cnn_all.pth
--cnn.pth
--cnn_23.pth
--crop_img.py
--2023Fheldout(test dataset)
----1.jpg
----2.jpg
----3.jpg
......
----300.jpg
----labels.txt

3.Please execute the command in terminal:

> python predict.py -d 2023Fheldout -m cnn_all

Please substitute "2023Fheldout" in the command to the name of test dataset, and substitute "cnn_all" in the command to the name of model

4.In this lab, we have trained several models, but Sean said we could only submit three models and the highest score would be taken. So please execute the following commands:

> python predict.py -d 2023Fheldout -m cnn_all
> python predict.py -d 2023Fheldout -m cnn
> python predict.py -d 2023Fheldout -m cnn_23

Thank you!