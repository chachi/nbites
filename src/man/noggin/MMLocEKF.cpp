#include "MMLocEKF.h"

// #define DEBUG_MERGING
// #define DEBUG_MERGING_THRESH
//#define DEBUG_SMALL_PROB

// @todo implement mostLikelyModel tracking
MMLocEKF::MMLocEKF() :
    LocSystem(),mostLikelyModel(0), numActive(0),
    numFree(MAX_MODELS)
{
    initModels();
}

MMLocEKF::~MMLocEKF()
{
    destroyModels();
}

/**
 * Create the fixed models for the filter
 */
void MMLocEKF::initModels()
{
    for (int i = 0; i < MAX_MODELS; ++i){
        models[i] = new LocEKF();
        models[i]->deactivate();

        models[i]->setProbability(0.0);

        modelList.push_back(models[i]);
    }
    numActive = 0;
    numFree = MAX_MODELS;
    activateModel(models[0]);
    models[0]->setProbability(1.0);
}

/**
 * Delete all the models on the heap.
 */
void MMLocEKF::destroyModels()
{
    for (int i = 0; i < MAX_MODELS; ++i){
        delete models[i];
    }
}

/**
 * Update localization according to the given odometry and visual observations.
 */
void MMLocEKF::updateLocalization(MotionModel u, std::vector<Observation> Z)
{
    // Apply time update
    timeUpdate(u);

    bool hasAppliedACorrection = correctionStep(Z);

    if (!hasAppliedACorrection)
        applyNoCorrectionStep();

    endFrame();

    lastObservations = Z;
    lastOdo = u;
}

void MMLocEKF::timeUpdate(MotionModel u)
{
    for (int i=0; i < MAX_MODELS; ++i)
        if ( models[i]->isActive() ){
            models[i]->odometryUpdate(u);
        }
}

bool MMLocEKF::correctionStep(vector<Observation>& Z)
{
    const bool appliedUnambiguous = applyUnambiguousObservations(Z);
    const bool appliedAmbiguous = applyAmbiguousObservations(Z);
    return appliedUnambiguous || appliedAmbiguous;
}

bool MMLocEKF::applyUnambiguousObservations(vector<Observation>& Z)
{
    bool appliedObs = false;
    vector<Observation>::iterator obs = Z.begin();
    while (obs != Z.end()){
        if (!obs->isAmbiguous()){
            applyObsToActiveModels(*obs);
            obs = Z.erase(obs);
            appliedObs = true;
        } else {
            ++obs;
        }
    }
    consolidateModels();
    return appliedObs;
}

void MMLocEKF::applyObsToActiveModels(const Observation& obs)
{
    for (int i=0; i < MAX_MODELS; ++i){
        if (models[i]->isActive()){
            models[i]->applyObservation(obs);
        }
    }
}

void MMLocEKF::applyNoCorrectionStep()
{
    for (int i=0; i < MAX_MODELS; ++i){
        if (models[i]->isActive()){
            models[i]->noCorrectionStep();
        }
    }
}

bool MMLocEKF::applyAmbiguousObservations(const vector<Observation>& Z)
{
#ifdef DEBUG_AMBIGUOUS
    cout << "\tApplying " << Z.size() << " ambiguous observations." << endl;
#endif /* DEBUG_AMBIGUOUS */

    bool applied = false;
    if (!Z.empty())
        applied = true;

    list<int> activeModels;
    for (int i=0; i < MAX_MODELS ; ++i){
        if (models[i]->isActive())
            activeModels.push_back(i);
    }

    list<int>::iterator model;
    vector<Observation>::const_iterator obs;
    for (obs = Z.begin(); obs != Z.end() ; ++obs){
        for (model = activeModels.begin(); model != activeModels.end(); ++model)
        {
#ifdef DEBUG_AMBIGUOUS
            cout << "Applying " << *obs << " to model " << *model << endl;
#endif /* DEBUG_AMBIGUOUS */

            splitObservation(*obs, models[*model]);
        }
    }
    return applied;
}

void MMLocEKF::splitObservation(const Observation& obs, LocEKF * model)
{
    const double originalProb = model->getProbability();
    list<LocEKF*> splitModels;

    // const int numRequiredModels = obs.getNumPossibilities() * numActive;
    // bool consolidated;
    // if (numRequiredModels > numFree)
    //  consolidated = consolidateModels(MAX_MODELS - numRequiredModels);

    // @TODO Better handling of too few free models
#ifdef DEBUG_AMBIGUOUS
    cout << "\tSplitting model " << *model << endl << "\tinto " <<
        obs.getNumPossibilities() << " models" << endl;
#endif /* DEBUG_AMBIGUOUS */

    for (unsigned int i=0; i < obs.getNumPossibilities(); ++i){
        if (numFree < 1){
            break;
        }

        LocEKF * inactiveModel = getInactiveModel();
        inactiveModel->copyEKF(*model);

        Observation newObs(obs);
        if (obs.isLine()){
            newObs.setLinePossibility(obs.getLinePossibilities()[i] );
        } else{
            newObs.setPointPossibility(obs.getPointPossibilities()[i] );
        }
#ifdef DEBUG_AMBIGUOUS
            cout << "SPLIT: Applying " << obs
                 << " to model " << *inactiveModel << endl;
#endif /* DEBUG_AMBIGUOUS */

        bool isOutlier = inactiveModel->applyObservation(newObs);
        if (!isOutlier) {
            activateModel(inactiveModel);
            splitModels.push_back(inactiveModel);
        }
    }

    // If every possibility is an outlier and the original model was the only
    // model to start with, than we need to keep this model active.
    if (splitModels.size() > 0) {
        deactivateModel(model);
        normalizeProbabilities(modelList, PROB_SUM);
        consolidateModels();
        normalizeProbabilities(modelList, PROB_SUM);
    } else {
        model->updateState();
    }
}

void MMLocEKF::endFrame()
{
    for (int i=0; i < MAX_MODELS; ++i){
        if (models[i]->isActive()){
            models[i]->endFrame();
        }
    }

    consolidateModels(MAX_ACTIVE_MODELS);
    normalizeProbabilities(modelList, PROB_SUM);
#ifdef DEBUG_ACTIVE_PROBS

#endif

#ifdef DEBUG_ACTIVE_PROBS
    if (numActive > 1){
    cout << "Model Probabilities:";
    for (int i=0; i < MAX_MODELS; ++i){
        if (models[i]->isActive()){
            cout << setprecision(3) << " " << models[i]->getProbability();
        }
    }
    cout << endl;
    }
#endif /* DEBUG_ACTIVE_PROBS */
}

void MMLocEKF::normalizeProbabilities(const list<LocEKF*>& unnormalized,
                                      double totalProb)
{
    // Normalize the Probabilities so that they all sum to totalProb
    double sumAlpha=0.0;
    list<LocEKF*>::const_iterator model = unnormalized.begin();
    for ( ; model != unnormalized.end() ; model++) {
        if ((*model)->isActive()) {
            sumAlpha += (*model)->getProbability();
        }
    }

    if(sumAlpha == 1.0) return;

    if (sumAlpha == 0) sumAlpha = 1e-8;

    model = unnormalized.begin();
    for ( ; model != unnormalized.end() ; model++) {
        if ((*model)->isActive()) {
            (*model)->setProbability((*model)->getProbability()/sumAlpha);
        }
    }

}

// @todo Tune merging and # of MAX_ACTIVE_MODELS
// Returns true if consolidation was successful
bool MMLocEKF::consolidateModels(int maxAfterMerge)
{

#ifdef DEBUG_MERGING
    cout << "Consolidating " << numActive << " models" << endl;
#endif /* DEBUG_MERGING */

    double mergeThreshold = MERGE_THRESH_INIT;
    int numMerges = 0;

    const int MAX_MERGES = 10;
    const double MIN_ACCEPT_PROB = 0.0001;

    for (int i=0; i < MAX_MODELS; ++i){
        if (models[i]->isActive() &&
            models[i]->getProbability() < MIN_ACCEPT_PROB){

#ifdef DEBUG_SMALL_PROB
            cout << "Deactivating model with prob. of " <<
                models[i]->getProbability() << endl;
#endif

            deactivateModel(models[i]);
        }
    }

    bool shouldMergeAgain = true;
    while ((numActive > maxAfterMerge || numMerges == 0) &&
           numMerges < 5){
        mergeThreshold += MERGE_THRESH_STEP;

#ifdef DEBUG_MERGING
        cout << "\n\nMerge #:" << numMerges <<
            " Thresh = " << mergeThreshold << endl;
#endif

        mergeModels(mergeThreshold);
        numMerges++;
    }

#ifdef DEBUG_MERGING
    cout << "\t" << numActive << " models remaining after consolidation: ";
    for (int i = 0; i < MAX_MODELS; ++i)
    {
        if (models[i]->isActive())
            cout << " " << i;
    }
    cout << endl;
#endif

    normalizeProbabilities(modelList, PROB_SUM);
    return true;
}

void MMLocEKF::mergeModels(double mergeThreshold)
{
    for (int i=0; i < MAX_MODELS; ++i) {
        for (int j=0; j < MAX_MODELS; ++j){
            if (i != j && mergeable(mergeThreshold,
                                    models[i], models[j])){

#ifdef DEBUG_MERGING
                cout << "Merging " << *models[i] << endl << " with " << endl
                     << *models[j] << endl;
#endif
                models[i]->mergeEKF(*models[j]);

#ifdef DEBUG_MERGING
                cout << "Merged model: " << *models[i] << endl;
#endif /* DEBUG_MERGING */
                deactivateModel(models[j]);
            }
        }
    }
}

bool MMLocEKF::mergeable(double mergeThreshold, LocEKF* one, LocEKF* two)
{
    if (!one->isActive() || !two->isActive() || one == two)
        return false;

    LocEKF::StateVector oneState = one->getState();
    LocEKF::StateVector twoState = two->getState();

    LocEKF::StateVector diff(3,0.0f);
    diff(0) = oneState(0) - twoState(0);
    diff(1) = oneState(1) - twoState(1);

    const float diffTheta = oneState(2) - twoState(2);
    diff(2) = atan2(sin(diffTheta), cos(diffTheta));

    LocEKF::StateMatrix oneUncert = one->getStateUncertainty();
    LocEKF::StateMatrix twoUncert = two->getStateUncertainty();

    LocEKF::StateMatrix uncertSum = (oneUncert * one->getProbability() +
                                     twoUncert * two->getProbability());


    LocEKF::StateMatrix uncertSumInv(3,3,0.0f);
    uncertSumInv(0,0) = 1./uncertSum(0,0);
    uncertSumInv(1,1) = 1./uncertSum(1,1);
    uncertSumInv(2,2) = 1./uncertSum(2,2);

    const double metric = abs(one->getProbability() * two->getProbability() *
                              inner_prod(trans(diff),
                                         prod(uncertSumInv, diff)));

#ifdef DEBUG_MERGING_THRESH
    cout << endl << endl << "\tMetric between " << endl
         << "\t" << *one << endl
         << "\tand" << endl << "\t" << *two << endl
         <<"\tis " << metric << endl << endl
         << "\tDiff is " << diff << endl
         << "\tInverse sum is" << uncertSumInv << endl;
    if (metric < mergeThreshold)
        cout << "\t\tMERGING" << endl;
    else
        cout << "\t\tNOT MERGING" << endl;

#endif

    return (metric < mergeThreshold);
}


/****************** Getters *****************************/
const PoseEst MMLocEKF::getCurrentEstimate() const
{
    return models[getMostLikelyModel()]->getCurrentEstimate();
}

const PoseEst MMLocEKF::getCurrentUncertainty() const
{
    return models[getMostLikelyModel()]->getCurrentUncertainty();
}

const float MMLocEKF::getXEst() const
{
    return models[getMostLikelyModel()]->getXEst();
}

const float MMLocEKF::getYEst() const
{
    return models[getMostLikelyModel()]->getYEst();
}

const float MMLocEKF::getHEst() const
{
    return models[getMostLikelyModel()]->getHEst();
}

const float MMLocEKF::getHEstDeg() const
{
    return models[getMostLikelyModel()]->getHEstDeg();
}

const float MMLocEKF::getXUncert() const
{
    return models[getMostLikelyModel()]->getXUncert();
}

const float MMLocEKF::getYUncert() const
{
    return models[getMostLikelyModel()]->getYUncert();
}

const float MMLocEKF::getHUncert() const
{
    return models[getMostLikelyModel()]->getHUncert();
}

const float MMLocEKF::getHUncertDeg() const
{
    return models[getMostLikelyModel()]->getHUncertDeg();
}

const list<LocEKF*> MMLocEKF::getModels() const
{
    return modelList;
}

// @todo implement checking to make sure index 0 isn't always called,
// or something wrong like that, aka better fall back behavior
const int MMLocEKF::getMostLikelyModel() const
{
    double max = -1.0;
    int index = 0;

    for (int i=0; i<MAX_MODELS; ++i){
        if (models[i]->getProbability() > max &&
            models[i]->isActive()){
            max = models[i]->getProbability();
            index = i;
        }
    }

    return index;
}

LocEKF * MMLocEKF::getInactiveModel() const
{
    for (int i=0; i < MAX_MODELS; ++i)
        if (!models[i]->isActive())
            return models[i];
    cout << "SHIT RAN OUT OF MODELS";
    exit(0);
    return models[MAX_MODELS -1];
}

/************** PRIVATE HELPERS ***********/
void MMLocEKF::setAllModelsInactive()
{
    for (int i=0; i < MAX_MODELS; ++i)
        deactivateModel(models[i]);
}

void MMLocEKF::equalizeProbabilities()
{
    const double newProbability = 1.0/numActive;
    for (int i=0; i < MAX_MODELS ; ++i){
        if (models[i]->isActive())
            models[i]->setProbability(newProbability);
    }
}

void MMLocEKF::deactivateModel(LocEKF * model)
{
    if (!model->isActive())
        return;
    model->deactivate();
    numActive--;
    numFree++;
}

void MMLocEKF::activateModel(LocEKF * model)
{
    if (model->isActive())
        return;
    model->activate();
    numActive++;
    numFree--;
}

/**************** RESETS *******************/
void MMLocEKF::blueGoalieReset()
{
    setAllModelsInactive();
    activateModel(models[0]);
    equalizeProbabilities();
    models[0]->blueGoalieReset();
}

void MMLocEKF::redGoalieReset()
{
    setAllModelsInactive();
    activateModel(models[0]);
    equalizeProbabilities();
    models[0]->redGoalieReset();
}

void MMLocEKF::reset()
{
    setAllModelsInactive();
    activateModel(models[0]);
    equalizeProbabilities();
    models[0]->reset();

}
