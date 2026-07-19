#!/usr/bin/env bash
# Create + register the Pi's IoT device certificate and drop the files into ../certs/
# so the new Pi just needs a `git pull` (certs are gitignored) OR a copy of ../certs.
#
# Run once, from any machine with the `skimr` profile. Then copy ../certs to the Pi
# (or re-run on the Pi). Idempotent-ish: creating a new cert each run is fine, but
# you'll accumulate certs in IoT - detach/delete old ones if you re-run.
set -euo pipefail

PROFILE=skimr
REGION=us-east-1
PROJECT=birdblast
THING=watergun-pi
POLICY=${PROJECT}-device
HERE="$(cd "$(dirname "$0")" && pwd)"
CERTS="$HERE/../certs"
AWS="aws --profile $PROFILE --region $REGION"

mkdir -p "$CERTS"
echo "==> Creating keys + certificate..."
CERT_JSON=$($AWS iot create-keys-and-certificate --set-as-active \
  --certificate-pem-outfile "$CERTS/watergun-pi.cert.pem" \
  --private-key-outfile "$CERTS/watergun-pi.private.key" \
  --public-key-outfile "$CERTS/watergun-pi.public.key")
CERT_ARN=$(echo "$CERT_JSON" | python3 -c "import json,sys;print(json.load(sys.stdin)['certificateArn'])")
echo "    cert ARN: $CERT_ARN"

echo "==> Attaching policy ($POLICY) + thing ($THING)..."
$AWS iot attach-policy --policy-name "$POLICY" --target "$CERT_ARN"
$AWS iot attach-thing-principal --thing-name "$THING" --principal "$CERT_ARN"

echo "==> Fetching Amazon Root CA 1..."
curl -s https://www.amazontrust.com/repository/AmazonRootCA1.pem -o "$CERTS/AmazonRootCA1.pem"

DATA_EP=$($AWS iot describe-endpoint --endpoint-type iot:Data-ATS --query endpointAddress --output text)
CRED_EP=$($AWS iot describe-endpoint --endpoint-type iot:CredentialProvider --query endpointAddress --output text)
IMAGES=$($AWS cloudformation describe-stacks --stack-name ${PROJECT}-core \
  --query "Stacks[0].Outputs[?OutputKey=='ImagesBucketName'].OutputValue" --output text)

echo ""
echo "Certs written to $CERTS/ (gitignored). Set these in the Pi's config.json 'cloud' section:"
echo "  \"enabled\": true,"
echo "  \"endpoint\": \"$DATA_EP\","
echo "  \"credentials_endpoint\": \"$CRED_EP\","
echo "  \"role_alias\": \"${PROJECT}-device-s3\","
echo "  \"images_bucket\": \"$IMAGES\","
echo "  \"region\": \"$REGION\""
echo ""
echo "Copy the whole ../certs directory to the Pi at <repo>/certs/ before starting the service."
