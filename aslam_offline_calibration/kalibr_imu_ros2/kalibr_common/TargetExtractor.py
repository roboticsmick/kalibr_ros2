import sm

import numpy as np
import sys
import multiprocessing
try:
   import queue
except ImportError:
   import Queue as queue # python 2.x
import time
import copy
import cv2

def multicoreExtractionWrapper(detector, taskq, resultq, clearImages, noTransformation):    
    while 1:
        try:
            task = taskq.get_nowait()
        except queue.Empty:
            return
        idx = task[0]
        stamp = task[1]
        image = task[2]
        
        if noTransformation:
            success, obs = detector.findTargetNoTransformation(stamp, np.array(image))
        else:
            success, obs = detector.findTarget(stamp, np.array(image))
            
        if clearImages:
            obs.clearImage()
        if success:
            resultq.put( (obs, idx) )

def extractCornersFromDataset(dataset, detector, multithreading=False, numProcesses=None, clearImages=True, noTransformation=False):
    print("Extracting calibration target corners")    
    targetObservations = []
    numImages = dataset.numImages()
    
    # prepare progess bar
    iProgress = sm.Progress2(numImages)
    iProgress.sample()
            
    if multithreading:
        if not numProcesses:
            numProcesses = max(1,multiprocessing.cpu_count()-1)
        try:
            # Use plain multiprocessing.Queue (not Manager.Queue) so items are
            # pickled/unpickled directly in the worker without a proxy server.
            # Manager.Queue double-proxies numpy arrays and breaks numpy_eigen
            # converters; it also spawns extra server processes unnecessarily.
            resultq = multiprocessing.Queue()
            taskq = multiprocessing.Queue()

            for idx, (timestamp, image) in enumerate(dataset.readDataset()):
                taskq.put( (idx, timestamp, image) )
                
            collected = []
            plist=list()
            for pidx in range(0, numProcesses):
                detector_copy = copy.copy(detector)
                p = multiprocessing.Process(target=multicoreExtractionWrapper, args=(detector_copy, taskq, resultq, clearImages, noTransformation, ))
                p.start()
                plist.append(p)

            # Drain the result queue continuously while waiting for workers.
            # This is required to prevent a deadlock: workers block trying to put
            # large observation objects into the queue once the OS pipe buffer
            # (~64 KB) fills up, but the main process only reads AFTER all workers
            # finish — a classic circular wait.
            last_done=0
            while 1:
                # Drain whatever is already available
                while True:
                    try:
                        collected.append(resultq.get_nowait())
                    except queue.Empty:
                        break
                if all([not p.is_alive() for p in plist]):
                    time.sleep(0.05)
                    break
                done = numImages-taskq.qsize()
                sys.stdout.flush()
                if (done-last_done) > 0:
                    iProgress.sample(done-last_done)
                last_done = done
                time.sleep(0.5)
            # Final drain after all workers have exited
            while True:
                try:
                    collected.append(resultq.get_nowait())
                except queue.Empty:
                    break
        except Exception as e:
            raise RuntimeError("Exception during multithreaded extraction: {0}".format(e))
        
        #get result sorted by time (=idx) — already collected above to prevent deadlock
        if collected:
            sortedObs = sorted(collected, key=lambda tup: tup[1])
            targetObservations = [obs for obs, idx in sortedObs]
        else:
            targetObservations=[]
    
    #single threaded implementation
    else:
        for timestamp, image in dataset.readDataset():
            if noTransformation:
                success, observation = detector.findTargetNoTransformation(timestamp, np.array(image))
            else:
                success, observation = detector.findTarget(timestamp, np.array(image))
            if clearImages:
                observation.clearImage()
            if success == 1:
                targetObservations.append(observation)
            iProgress.sample()

    if len(targetObservations) == 0:
        print("\r")
        sm.logFatal("No corners could be extracted for camera {0}! Check the calibration target configuration and dataset.".format(dataset.topic))
    else:    
        print("\r  Extracted corners for %d images (of %d images)                              " % (len(targetObservations), numImages))

    #close all opencv windows that might be open
    cv2.destroyAllWindows()
    
    return targetObservations
