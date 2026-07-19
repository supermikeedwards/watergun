// Sample of the runtime config the SPA expects. The REAL web/config.js is generated
// by infra/deploy.sh from the CloudFormation stack outputs and is gitignored.
// Copy this to config.js and fill in for local dev if needed.
window.BIRDBLAST = {
  region: "us-east-1",
  userPoolId: "us-east-1_XXXXXXXXX",
  userPoolClientId: "xxxxxxxxxxxxxxxxxxxxxxxxxx",
  identityPoolId: "us-east-1:xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx",
  hostedUiDomain: "https://birdblast-872515289389.auth.us-east-1.amazoncognito.com",
  iotEndpoint: "xxxxxxxxxxxxxx-ats.iot.us-east-1.amazonaws.com",
  imagesBucket: "birdblast-images-872515289389",
  thingName: "watergun-pi",
  domain: "https://birdblast.the-edwards.fr"
};
