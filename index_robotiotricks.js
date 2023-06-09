// This sample demonstrates handling intents from an Alexa skill using the Alexa Skills Kit SDK (v2).
// Please visit https://alexa.design/cookbook for additional examples on implementing slots, dialog management,
// session persistence, api calls, and more.
const Alexa = require('ask-sdk-core');
const mqttHelper = require('./mqtthelper');

const LaunchRequestHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'LaunchRequest';
    },
    handle(handlerInput) {
        const speakOutput = 'Welcome to the robot of internet of tricks, you can ask me to move your robot in any direction. Which would you like to try? And how far you want to go?';
        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};
const MoveIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'MoveIntent';
    },
    async handle(handlerInput) {
        const commandVal = handlerInput.requestEnvelope.request.intent.slots.command.value
        valueVal = handlerInput.requestEnvelope.request.intent.slots.value.value
        metricVal = handlerInput.requestEnvelope.request.intent.slots.metric.value
        if (valueVal == undefined || metricVal == undefined){
            if (commandVal == "forward" || commandVal == "back"){
                metricVal = "centimeters"
                valueVal = 10;
            }else{
                metricVal = "degrees"
                valueVal = 90;
            }

        }
        const speakOutput = `Moving ${commandVal} ${valueVal} ${metricVal}`
        
        return mqttHelper.robotCommand(commandVal,valueVal).then((data) => {
            if (data.success) {
                return handlerInput.responseBuilder
                    .speak(speakOutput)
                    .reprompt('')
                    .getResponse();
            } else {
                speechText = 'Error while sending MQTT for smart car'
                return handlerInput.responseBuilder
                    .speak(speechText)
                    .getResponse();
            }
        })
    }
};
const PauseIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'PauseIntent';
    },
    async handle(handlerInput) {
        const speakOutput = 'Stopping now';
        return mqttHelper.robotCommand('stop',0).then((data) => {
            if (data.success) {
                return handlerInput.responseBuilder
                    .speak(speakOutput)
                    .reprompt('')
                    .getResponse();
            } else {
                speechText = 'Error while sending MQTT for smart car'
                return handlerInput.responseBuilder
                    .speak(speechText)
                    .getResponse();
            }
        })
    }
};
const HelpIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.HelpIntent';
    },
    handle(handlerInput) {
        const speakOutput = 'You can say hello to me! How can I help?';

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};
const CancelAndStopIntentHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest'
            && (Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.CancelIntent'
                || Alexa.getIntentName(handlerInput.requestEnvelope) === 'AMAZON.StopIntent');
    },
    async handle(handlerInput) {
        const speakOutput = 'Stopping now';
        return mqttHelper.robotCommand('stop',0).then((data) => {
            if (data.success) {
                return handlerInput.responseBuilder
                    .speak(speakOutput)
                    .getResponse();
            } else {
                speechText = 'Error while sending MQTT for smart car'
                return handlerInput.responseBuilder
                    .speak(speechText)
                    .getResponse();
            }
        })
    }
};
const SessionEndedRequestHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'SessionEndedRequest';
    },
    handle(handlerInput) {
        // Any cleanup logic goes here.
        return handlerInput.responseBuilder.getResponse();
    }
};

// The intent reflector is used for interaction model testing and debugging.
// It will simply repeat the intent the user said. You can create custom handlers
// for your intents by defining them above, then also adding them to the request
// handler chain below.
const IntentReflectorHandler = {
    canHandle(handlerInput) {
        return Alexa.getRequestType(handlerInput.requestEnvelope) === 'IntentRequest';
    },
    handle(handlerInput) {
        const intentName = Alexa.getIntentName(handlerInput.requestEnvelope);
        const speakOutput = `You just triggered ${intentName}`;

        return handlerInput.responseBuilder
            .speak(speakOutput)
            //.reprompt('add a reprompt if you want to keep the session open for the user to respond')
            .getResponse();
    }
};

// Generic error handling to capture any syntax or routing errors. If you receive an error
// stating the request handler chain is not found, you have not implemented a handler for
// the intent being invoked or included it in the skill builder below.
const ErrorHandler = {
    canHandle() {
        return true;
    },
    handle(handlerInput, error) {
        console.log(`~~~~ Error handled: ${error.stack}`);
        const speakOutput = `Sorry, I had trouble doing what you asked. Please try again.`;

        return handlerInput.responseBuilder
            .speak(speakOutput)
            .reprompt(speakOutput)
            .getResponse();
    }
};

// The SkillBuilder acts as the entry point for your skill, routing all request and response
// payloads to the handlers above. Make sure any new handlers or interceptors you've
// defined are included below. The order matters - they're processed top to bottom.
exports.handler = Alexa.SkillBuilders.custom()
    .addRequestHandlers(
        LaunchRequestHandler,
        MoveIntentHandler,
        PauseIntentHandler,
        HelpIntentHandler,
        CancelAndStopIntentHandler,
        SessionEndedRequestHandler,
        IntentReflectorHandler, // make sure IntentReflectorHandler is last so it doesn't override your custom intent handlers
        ) 
    .addErrorHandlers(
        ErrorHandler,
        )
    .lambda();
