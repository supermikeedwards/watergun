# Birdblast AWS infra (worklog §18)

Turns the Pi into a thin AWS IoT client and serves the **birdblast** SPA + detection
images from AWS, so the site works whether or not the Pi is online. All in **us-east-1**,
account **872515289389**, profile **`skimr`**.

```
Browser -- CloudFront -- S3 (SPA)               <- app shell + assets
   |            \------- S3 (images, /images/*)  <- detection gallery
   |-- Cognito User Pool (login) + Identity Pool (scoped AWS creds)
   |-- AWS IoT Core (MQTT/WSS): Device Shadow (config/state) + watergun/* topics
   \-- (images read direct from S3 via Identity Pool creds)
Pi  -- one outbound MQTT/TLS conn (cert) -- same IoT Core + S3 upload via cred provider
```

## Stacks
- **`01-core.yaml`** (`birdblast-core`): IoT Thing + device policy + role alias, Cognito
  user/identity pools, S3 buckets (web + images), IAM (browser auth role, device S3 role).
- **`02-cdn.yaml`** (`birdblast-cdn`): ACM cert (DNS-validated via Route53), CloudFront
  (SPA default + `/images/*`), bucket policies, Route53 ALIAS `birdblast.the-edwards.fr`.

> No Lambda@Edge JWT gate in v1: the data plane (IoT + S3) is already locked to logged-in
> users by the Identity Pool IAM role, and the static shell holds no secrets. Edge
> shell-gating is a documented phase-2 hardening.

## Deploy (one command)
```bash
cd infra
./deploy.sh          # deploys both stacks, uploads the SPA, prints all IDs
```
The ACM cert validates automatically because the Route53 zone is in the same account
(first run waits a few minutes on the CDN stack). `deploy.sh` writes `web/config.js` from
the stack outputs and syncs `web/` to the SPA bucket.

Re-upload just the SPA after an edit:
```bash
./deploy.sh --spa-only
```

## Register the Pi (device certificate)
```bash
cd infra
./provision-device-cert.sh    # creates + activates a cert, attaches policy + thing,
                              # downloads certs into ../certs/, prints the config values
```
Copy `../certs/` to the Pi at `<repo>/certs/` (gitignored - never committed), then set the
printed values into the Pi's `config.json` `"cloud"` block and set `"enabled": true`.

## Create the login user (once)
```bash
aws --profile skimr --region us-east-1 cognito-idp admin-create-user \
  --user-pool-id <UserPoolId> --username you@example.com \
  --user-attributes Name=email,Value=you@example.com Name=email_verified,Value=true
aws --profile skimr --region us-east-1 cognito-idp admin-set-user-password \
  --user-pool-id <UserPoolId> --username you@example.com \
  --password 'a-strong-password' --permanent
```

## Record the identifiers
After `deploy.sh`, paste its summary block into worklog **§18.6 (W7)** so the Cognito
IDs / IoT endpoints / bucket names are captured (and reusable for Skimr).

## Teardown
`aws cloudformation delete-stack` for `birdblast-cdn` then `birdblast-core` (empty the S3
buckets first). Detach/delete the device cert from IoT if you re-provision.
